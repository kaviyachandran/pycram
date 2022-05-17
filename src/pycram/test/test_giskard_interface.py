from unittest import TestCase

import giskard_msgs.msg

from pycram.bullet_world import BulletWorld, Object
from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.giskard_interface import BaseGoal

from geometry_msgs.msg import PoseStamped, Vector3

import rospkg
import rospy, actionlib

from giskard_msgs.msg import MoveAction

### set up btw with PR2 and kitchen
### create an action server
### make base goal
### send goal
##class TestGiskardInterface(TestCase):

def setUp():
    pass

# self.setup_action_server()
# for joint, pose in robot_description.i.get_static_joint_chain("right", "park").items():
# 	robot.set_joint_state(joint, pose)
#
# for joint, pose in robot_description.i.get_static_joint_chain("left", "park").items():
# 	robot.set_joint_state(joint, pose)
#
# robot.set_position_and_orientation([0.3, 0.0, 0], [0, 0, 0, 1])

def tearDown() -> None:
    pass


def test_make_base_goal():
    world = BulletWorld()
    world.set_gravity([0, 0, -9.8])

    ##path = rospkg.RosPack().get_path('pycram')
    ##print(path)
    robot = Object("pr2", "robot", "pr2.urdf")
    env = Object("kitchen", "environment", "kitchen.urdf")
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = 0
    goal_pose.pose.position = Vector3(0.5, 0, 0)
    velocity_type = "slow"

    r_joints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
                "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint",
                "r_wrist_roll_joint"]

    l_joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint",
                "l_wrist_roll_joint"]

    base_goal = BaseGoal(goal_pose=goal_pose, base_velocity_type=velocity_type,
                         left_arm_joints=l_joints, right_arm_joints=r_joints)
    base_goal.make_goal()
    base_goal.send_base_goal()

    ##self.assertEqual(True, True)

# def tearDown(self) -> None:
#     pass
#
#
# def setup_action_server(self):
#     result = giskard_msgs.msg.MoveResult()
#
#     def __init__(self):
#         self.server = actionlib.SimpleActionServer('/giskard/command', MoveAction,
#                                                    execute_cb=self.execute_cb)
#         self.server.start()
#
#     def execute_cb(self, goal):
#         self.server.set_succeeded(self.result)


if __name__ == "__main__":
    setUp()
    test_make_base_goal()