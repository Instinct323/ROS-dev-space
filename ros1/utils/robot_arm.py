import copy
import itertools
import sys
from typing import Callable

import actionlib
import franka_gripper.msg
import geometry_msgs.msg
import moveit_commander
import numpy as np
import ros_numpy
import rospy


class Franka:
    """
    A class to control a robot arm using MoveIt! in ROS.
    """

    def __init__(self,
                 arm_id: str = "panda",
                 planner_id: str = "RRTConnect",
                 timeout: float = 5.0):
        self.timeout = rospy.Duration(timeout)

        # frame ID
        self.arm_id = arm_id
        self.base_link = f"{arm_id}_link0"
        self.ee_link = f"{arm_id}_hand"

        # robot commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        # print("Group names:", self.robot.get_group_names())

        # planning scene interface
        self.scene = moveit_commander.PlanningSceneInterface()

        # MoveIt! group for the arm
        self.arm_group = moveit_commander.MoveGroupCommander(f"{arm_id}_arm", wait_for_servers=timeout)
        self.arm_group.set_planning_time(timeout)
        self.arm_group.set_planner_id(planner_id)

        # action client for the gripper
        self.hand_client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        self.hand_client.wait_for_server(self.timeout)

    @property
    def Teb(self) -> np.ndarray:
        return ros_numpy.numpify(self.arm_group.get_current_pose(self.ee_link).pose)

    def update_scene(self,
                     Twb: np.ndarray,
                     plane_height: float = 0.14,
                     obj_pcd: list[np.ndarray] = None):
        self.scene.remove_world_object()

        # add a plane
        if plane_height is not None:
            assert plane_height <= 0.14
            pose_msg = geometry_msgs.msg.PoseStamped()
            pose_msg.header.frame_id = self.base_link
            self.scene.add_plane("ground", pose=pose_msg, offset=plane_height)

        # add point clouds
        if obj_pcd is not None:
            for i, pcd in enumerate(obj_pcd):
                if pcd.size == 0: continue
                pcd = (Twb[:3, :3] @ pcd.T).T + Twb[:3, 3]
                bbox = np.stack([pcd.min(axis=0), pcd.max(axis=0)])

                pose_msg = geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(
                    position=ros_numpy.msgify(geometry_msgs.msg.Point, bbox.mean(axis=0)),
                ))
                pose_msg.pose.orientation.w = 1.
                pose_msg.header.frame_id = self.base_link
                self.scene.add_box(f"pcd{i}", pose=pose_msg, size=bbox[1] - bbox[0])

    def move(self,
             T_ee_base: np.ndarray,
             wait: bool = True,
             n_seg: int = 0) -> bool:
        """
        Moves the end effector to the specified pose in the base frame
        :param T_ee_base: 4x4 transformation matrix from the end effector to the base frame
        """
        pose_msg = ros_numpy.msgify(geometry_msgs.msg.Pose, T_ee_base)
        self.arm_group.set_pose_target(pose_msg, end_effector_link=self.ee_link)

        # try to plan the trajectory
        success, plan, planning_time, error_code = self.arm_group.plan()
        self.arm_group.clear_pose_targets()

        if success:
            if n_seg == 0:
                return self.arm_group.execute(plan, wait=wait)

            # segment the trajectory if requested
            elif n_seg > 0:
                wpt = plan.joint_trajectory.points
                n_wpt = len(wpt)
                n_seg = min(n_seg, n_wpt)
                indices = np.round(np.linspace(0, n_wpt - 1, n_seg + 1, endpoint=True)).astype(int)
                for i in indices[1:]:

                    # create a new plan for the segment
                    self.arm_group.set_joint_value_target(wpt[i].positions)
                    success_i, plan_i, *_ = self.arm_group.plan()
                    self.arm_group.clear_pose_targets()
                    if not success_i: break

                    # query the user for confirmation
                    execute = "s"
                    while execute not in "yn":
                        execute = input("Execute the next segment? (y/n): ").strip().lower()
                    if execute == "n": return False

                    # execute the segment
                    if not self.arm_group.execute(plan_i, wait=True): break
                    if i + 1 == n_wpt: return True

        rospy.logwarn("Failed to plan the trajectory.")
        return False

    def move_straight(self,
                      Ts_ee_base: list[np.ndarray],
                      success_ratio: float,
                      eef_step: float = 0.02,
                      wait: bool = True):
        pose_msgs = [ros_numpy.msgify(geometry_msgs.msg.Pose, T) for T in Ts_ee_base]
        plan, frac = self.arm_group.compute_cartesian_path(pose_msgs, eef_step=eef_step)

        if frac < success_ratio:
            rospy.logwarn(f"Failed to plan the trajectory with success ratio {frac:.2f}.")
            return False
        return self.arm_group.execute(plan, wait=wait)

    def grip(self,
             width: float,
             speed: float = 0.05,
             wait: bool = True) -> bool:
        """
        Sets the gripper width to the specified value.
        :param width: desired gripper width (in meters)
        """
        goal = franka_gripper.msg.GraspGoal(width=width, speed=speed)
        self.hand_client.send_goal(goal)
        assert wait and self.hand_client.wait_for_result(self.timeout)
        return True

    def estimate_ws(self,
                    dst_file: str,
                    f_pos2pose: Callable,
                    center: np.ndarray,
                    azim_limit: tuple[float, float] = (-np.pi, np.pi),
                    elev_limit: tuple[float, float] = (0, np.pi / 2 - 0.01),
                    tol_pos: float = 0.01,
                    tol_rad: float = 0.06,
                    radius_min: float = 0.1):
        """
        Estimates the workspace point cloud of the robot arm's end effector and saves it to a file.
        NOTE: Execute `roslaunch panda_moveit_config demo.launch robot:=panda` to start the MoveIt node.

        :param dst_file: path to save the point cloud (.npy file)
        :param f_pos2pose: function that takes a spatial point and returns a 4x4 pose transformation matrix
        :param center: center point of the workspace (3,)
        :param azim_limit: azimuth angle range (rad)
        :param elev_limit: elevation angle range (rad)
        :param tol_pos: position increment/tolerance
        :param tol_rad: angle increment/tolerance
        :param radius_min: minimum distance from the center
        """
        from tqdm import trange
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap("rainbow")

        # generate sampled azimuth and elevation angles
        azim = np.linspace(*azim_limit, round(np.diff(azim_limit).item() / tol_rad))
        elev = np.linspace(*elev_limit, round(np.diff(elev_limit).item() / tol_rad))
        azim, elev = map(np.ravel, np.meshgrid(azim, elev))

        # shuffle the angles
        n = len(azim)
        i = np.random.permutation(n)
        azim, elev = azim[i], elev[i]

        # unit direction vectors for each sample
        lookat = np.stack([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), - np.sin(elev)], axis=-1)
        radius = np.zeros(n)

        # incrementally increase radius and check reachability
        fig = plt.subplot(projection="3d")
        tbar = trange(n, desc="Estimating workspace")
        for i in tbar:
            for r in itertools.count(radius_min, tol_pos):
                if rospy.is_shutdown(): return

                pos = center - lookat[i] * r
                T_ee_base = f_pos2pose(pos)

                pose_msg = ros_numpy.msgify(geometry_msgs.msg.Pose, T_ee_base)
                self.arm_group.clear_pose_targets()
                self.arm_group.set_pose_target(pose_msg, end_effector_link="panda_hand")

                # record the last reachable radius
                if not self.arm_group.plan()[0]: break
                radius[i] = r

            # update the progress bar
            valid_rate = (radius[:i + 1] > 0).mean()
            tbar.set_postfix({"valid_rate": round(valid_rate, 4)})

            # compute all reachable spatial coordinates
            pcd = center - lookat[:i + 1] * radius[:i + 1, None]
            pcd = pcd[radius[:i + 1] > 0]
            np.save(dst_file, pcd)

            # visualize the current solution
            plt.cla()
            fig.scatter(*pcd.T, s=20, c=cmap(pcd[:, 2]))
            fig.view_init(elev=60, azim=-120)
            plt.axis("equal")
            plt.pause(1e-3)


if __name__ == '__main__':
    rospy.init_node("test_node", sys.argv)

    arm = Franka()

    waypoints = [arm.Teb]
    for i in range(3):
        T = waypoints[-1].copy()
        T[2, 3] += 0.05
        arm.move(T)

    for i in range(2):
        arm.grip(0.)
        arm.grip(0.035)
