import sys
import threading
from typing import List

import actionlib
import franka_gripper.msg
import franka_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import numpy as np
import ros_numpy
import rospy
import sensor_msgs.msg
import torch

from sym_lnk import curobo_planning as cuplan

FORCE_LIMIT = 15

# Pseudo-constants
ROBOT_MODE_OTHER = 0
ROBOT_MODE_IDLE = 1
ROBOT_MODE_MOVE = 2
ROBOT_MODE_GUIDING = 3
ROBOT_MODE_REFLEX = 4
ROBOT_MODE_USER_STOPPED = 5
ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY = 6


class FrankaROScontroller:
    robot_mode: int = property(lambda self: self.franka_state.robot_mode if self.franka_state else None)

    def __init__(self,
                 timeout: float = 3.):
        self.timeout = rospy.Duration(timeout)
        self.planner = cuplan.CuRoboPlanner(robot_cfg="franka.yml")
        self.idle = threading.Event()
        self.idle.set()

        # latest state
        self.franka_state: franka_msgs.msg.FrankaState = None
        self.width: float = None
        self.q: List[float] = None
        self.F_ext: List[float] = None

        # subscriber
        rospy.Subscriber("/franka_state_controller/franka_states",
                         franka_msgs.msg.FrankaState,
                         lambda msg: setattr(self, "franka_state", msg))
        rospy.Subscriber("/franka_gripper/joint_states",
                         sensor_msgs.msg.JointState,
                         lambda msg: setattr(self, "width", sum(msg.position)))
        rospy.Subscriber("/franka_state_controller/joint_states",
                         sensor_msgs.msg.JointState,
                         lambda msg: setattr(self, "q", msg.position))
        rospy.Subscriber("/franka_state_controller/F_ext",
                         geometry_msgs.msg.WrenchStamped,
                         lambda msg: setattr(self, "F_ext", ros_numpy.numpify(msg.wrench.force).tolist()))

        # franka_control / franka_gazebo
        self.gripper_grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        self.gripper_move_client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        self.error_recovery_client = actionlib.SimpleActionClient("/franka_control/error_recovery", franka_msgs.msg.ErrorRecoveryAction)

        # panda_moveit_config
        self.execute_traj_client = actionlib.SimpleActionClient("/execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction)

        # initialization
        rospy.loginfo("Waiting for action servers...")
        for client in (self.gripper_grasp_client, self.gripper_move_client, self.error_recovery_client, self.execute_traj_client):
            client.wait_for_server(self.timeout)

        rospy.loginfo("Waiting for initial state...")
        while any(
                x is None for x in (self.franka_state, self.width, self.q, self.F_ext)
        ): rospy.sleep(0.1)

        rospy.loginfo("Waiting for robot to be ready...")
        self._try_recover_from_error()
        threading.Thread(target=self.monitor_robot_state, daemon=True).start()
        self.grip(True)
        rospy.loginfo(f"Initialized: {self}")

    def __bool__(self):
        return self.robot_mode not in (ROBOT_MODE_GUIDING, ROBOT_MODE_USER_STOPPED, ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY)

    def __repr__(self):
        attr = {"robot_mode": "%d", "width": "%.4f"}
        desc = [f"{key}={fmt}" % getattr(self, key) for key, fmt in attr.items()]
        desc.append("q=%s" % np.array2string(np.array(self.q), precision=4, separator=","))
        return f"{__class__.__name__}({', '.join(desc)})"

    def _try_recover_from_error(self):
        now = rospy.Time.now()
        while True:
            rospy.sleep(0.02)
            if self.franka_state.header.stamp < now: continue
            if self.robot_mode == ROBOT_MODE_MOVE: return
            if self: break
        self.error_recovery_client.send_goal(franka_msgs.msg.ErrorRecoveryGoal())
        # the error recovery may not response if the robot is not in error mode
        # and there is no reliable way to check if the robot is in error mode
        # so we just wait for a fixed time
        rospy.loginfo("Waiting for error recovery to finish...")
        self.error_recovery_client.wait_for_result()
        rospy.loginfo("Error recovery finished")

    def monitor_robot_state(self):
        try:
            while True:
                rospy.sleep(0.1)
                if self.F_ext[2] > FORCE_LIMIT or self.robot_mode != ROBOT_MODE_MOVE:
                    self.idle.clear()
                    rospy.loginfo(f"large external force or robot abnormal, trying to recover...")
                    self._try_recover_from_error()
                    self.idle.set()
                    rospy.loginfo(f"robot recovered")
        except rospy.ROSInterruptException:
            pass

    def update_scene(self,
                     world_config: str | dict | cuplan.WorldConfig = None):
        self.planner.world_cfg = world_config

    def move(self,
             T_ee_base: np.ndarray,
             show: bool = False) -> bool:
        self.idle.wait()
        while not self: rospy.sleep(0.02)

        plan = self.planner.plan(q_start=torch.tensor(self.q)[None],
                                 goal=cuplan.Pose.from_matrix(T_ee_base[None]),
                                 vis_cfg=cuplan.PlanVisConfig() if show else None)
        if not plan: return False

        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory.joint_trajectory = cuplan.to_ros_plan(plan, self.planner.interpolation_dt)

        self.execute_traj_client.send_goal(goal)
        self.execute_traj_client.wait_for_result()
        result: moveit_msgs.msg.ExecuteTrajectoryResult = self.execute_traj_client.get_result()
        return result.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS

    def grip(self,
             is_open: bool):
        """ 
        Open or close the gripper
        FROM: https://github.com/MiYanDoris/GraspVLA-real-world-controller/blob/main/vla_client/controllers/franka_ros_controller.py 
        """
        self.idle.wait()
        # is_open_cur = abs(self.width - 0.08) < 0.001
        # if is_open == is_open_cur: return
        if is_open:
            goal = franka_gripper.msg.MoveGoal(width=0.08, speed=0.1)
            self.gripper_move_client.send_goal(goal)
            self.gripper_move_client.wait_for_result(timeout=self.timeout)
        else:
            goal = franka_gripper.msg.GraspGoal(width=0., speed=0.2, force=30)
            goal.epsilon.inner = 0.04
            goal.epsilon.outer = 0.04
            self.gripper_grasp_client.send_goal(goal)
            self.gripper_grasp_client.wait_for_result(timeout=self.timeout)


if __name__ == "__main__":
    # logical
    # - roslaunch franka_gazebo panda.launch
    # - roslaunch panda_moveit_config demo_gazebo.launch load_gripper:=true robot:=panda

    # physical
    # - roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda
    # - roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda

    from tf_manager import TFmanager, SE3

    rospy.init_node("franka_ctrl", sys.argv)
    TFmanager.load_yaml("../../config/franka_tran.yaml")

    # Initialize controller
    ctrl = FrankaROScontroller()
    ctrl.update_scene("collision_cage.yml")

    # T_ee_base
    init_Teb = [0.873475, -0.083195, 0.479709, 0.2997, -0.044218, -0.994776, -0.092009, 0.0056,
                0.484858, 0.059156, -0.87259, 0.655018]
    ctrl.move(SE3.from_config(init_Teb))

    # T_grasp_cam (to execute)
    TGc = [0.541297972202301, 0.8403080105781555, -0.02963699959218502, 0.1365479975938797,
           -0.5598459839820862, 0.3864839971065521, 0.7329409718513489, -0.05429200083017349,
           0.6273499727249146, -0.3801470100879669, 0.6796460151672363, 0.4324619960784912]
    ctrl.move(TFmanager["camera", "base"] @ SE3.from_config(TGc) @ TFmanager["hand", "grasp"])

    ctrl.grip(False)
