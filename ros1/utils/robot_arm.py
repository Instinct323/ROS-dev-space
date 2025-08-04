import sys

import actionlib
import franka_gripper.msg
import geometry_msgs.msg
import moveit_commander
import numpy as np
import ros_numpy


def plan_and_execute(move_group: moveit_commander.MoveGroupCommander,
                     wait: bool = True):
    """
    Plans and executes a trajectory using the given MoveGroupCommander.
    """
    success, plan, planning_time, error_code = move_group.plan()
    assert success, "Failed to plan the trajectory."
    move_group.execute(plan, wait=wait)
    move_group.clear_pose_targets()


class Franka:
    """
    A class to control a robot arm using MoveIt! in ROS.
    :param arm_group: name of the MoveIt! group for the arm
    :param hand_ns:
    """

    def __init__(self,
                 arm_group: str = "panda_arm",
                 hand_ns: str = "/franka_gripper/grasp"):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        # print("Group names:", self.robot.get_group_names())
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group = moveit_commander.MoveGroupCommander(arm_group)
        self.hand_client = actionlib.SimpleActionClient(hand_ns, franka_gripper.msg.GraspAction)
        self.hand_client.wait_for_server(timeout=rospy.Duration(5.0))

    def move(self,
             T_ee_base: np.ndarray,
             ee_link: str,
             wait: bool = True):
        """
        Moves the end effector to the specified pose in the base frame
        :param T_ee_base: 4x4 transformation matrix from the end effector to the base frame
        :param ee_link: name of the end effector link
        """
        pose_msg = ros_numpy.msgify(geometry_msgs.msg.Pose, T_ee_base)
        self.arm_group.set_pose_target(pose_msg, end_effector_link=ee_link)
        plan_and_execute(self.arm_group, wait=wait)

    def grip(self,
             width: float,
             speed: float = 0.05,
             wait: bool = True):
        """
        Sets the gripper width to the specified value.
        :param width: desired gripper width (in meters)
        """
        goal = franka_gripper.msg.GraspGoal()
        goal.width = width
        goal.speed = speed

        self.hand_client.send_goal(goal)
        assert wait and self.hand_client.wait_for_result()


if __name__ == '__main__':
    import rospy
    from tf_manager import TFmanager

    rospy.init_node("test_node", sys.argv)

    pose_cur = TFmanager["panda_hand", "panda_link0"]
    pose_cur[:3, 3] -= 0.05
    print(pose_cur)

    arm = Franka()
    arm.move(pose_cur, ee_link="panda_hand")
    for i in range(5):
        arm.grip(0.07)
        arm.grip(0.)
