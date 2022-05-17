from abc import abstractmethod
from typing import Optional

import actionlib
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped, PointStamped, Quaternion
from sensor_msgs.msg import JointState

from pycram.robot_description import InitializedRobotDescription as robot_description
from giskard_msgs.msg import MoveCmd, MoveAction, MoveGoal, MoveResult, Constraint, CollisionEntry
from pycram.bullet_world import BulletWorld, Object


class GiskardGoal():
    ### TODO : get environment name and world frame
    world = 'map'
    environment_name = 'kitchen'
    avoid_joint_limits_percentage = 40
    prefer_base_low_cost = 0.0001
    avoid_collisions_distance = 0.10
    unmovable_joint_weight = 9001
    collision_avoidance_hint_threshold = 0.25
    collision_avoidance_hint_spring_offset = 0.05
    collision_avoidance_hint_velocity = 1.0

    @abstractmethod
    def make_goal(self):
        pass

    def send_goal(self, goal):
        client = actionlib.SimpleActionClient('/giskard/command', MoveAction)
        client.wait_for_server()

        action_goal = MoveGoal()
        action_goal.type = MoveGoal.PLAN_AND_EXECUTE
        action_goal.cmd_seq.append(goal)
        client.send_goal(action_goal)
        client.wait_for_result()
        result = client.get_result()

        if result.error_codes[0] == MoveResult.SUCCESS:
            print('giskard returned success')
        elif result.error_codes[0] == MoveResult.UNREACHABLE:
            print('unreachable goal')
        else:
            print('something went wrong, error code :', result.error_codes[0])

    def get_head_pointing_constraint(self, goal_pose: PoseStamped) -> Constraint:
        ### TODO : Raise an exception or an info when the camera frame is not available
        return self.get_pointing_constraint(tip_frame=robot_description.i.get_camera_frame(),
                                            goal_pose=goal_pose, root_frame=goal_pose.header.frame_id, pointing_vector=None)

    def get_pointing_constraint(self, root_frame: str, tip_frame: str, goal_pose: PoseStamped,
                                pointing_vector: Optional[PoseStamped]) -> Constraint:

        goal_point = self._get_point_stamped(goal_pose)
        pointing_constraint = Constraint()
        pointing_constraint.type = 'Pointing'
        pointing_constraint.parameter_value_pair = {'root_link': root_frame,
                                                    'tip_link': tip_frame,
                                                    'goal_point': goal_point}
        if pointing_vector:
            pointing_constraint.parameter_value_pair['pointing_axis'] = pointing_vector

        return pointing_constraint

    def get_base_velocity_constraint(self, max_linear_velocity: float, max_angular_velocity: float) -> Constraint:
        velocity_constraint = Constraint()
        velocity_constraint.type = 'CartesianVelocityLimit'
        velocity_constraint.parameter_value_pair = {'root_link': robot_description.i.odom_frame,
                                                    'tip_link': robot_description.i.base_frame,
                                                    'max_linear_velocity': max_linear_velocity,
                                                    'max_angular_velocity': max_angular_velocity}

    def get_collision_avoidance_hint_constraint(self, environment_link: str, avoidance_hint: Vector3Stamped,
                                                threshold: collision_avoidance_hint_threshold,
                                                spring_offset: collision_avoidance_hint_spring_offset,
                                                max_velocity: collision_avoidance_hint_velocity) -> Constraint:

        collision_avoidance_constraint = Constraint()
        collision_avoidance_constraint.type = 'CollisionAvoidanceHint'
        collision_avoidance_constraint.parameter_value_pair = {'tip_link': robot_description.i.base_link,
                                                               'avoidance_hint': avoidance_hint,
                                                               'max_threshold': threshold,
                                                               'max_linear_velocity': max_velocity,
                                                               'spring_threshold': spring_offset,
                                                               'object_name': self.environment_name,
                                                               'object_link_name': environment_link,
                                                               'weight': Constraint.WEIGHT_COLLISION_AVOIDANCE}

        return collision_avoidance_constraint

    def get_cartesian_constraint(self, root_frame: str, tip_frame: str, goal_pose: PoseStamped,
                                 max_velocity: float, avoid_collisions: bool) -> Constraint:

        weight = Constraint.WEIGHT_BELOW_CA if avoid_collisions else None

        cartesian_constraint = Constraint()
        cartesian_constraint.type = 'CartesianPose'
        cartesian_constraint.parameter_value_pair = {'root_link': root_frame,
                                                     'tip_link': tip_frame,
                                                     'goal': goal_pose,
                                                     'max_linear_velocity': max_velocity,
                                                     'weight': weight}
        return cartesian_constraint

    def get_simple_joint_constraint(self, joint_state) -> Constraint:
        goal_state = JointState()
        goal_state.name = joint_state.keys()
        goal_state.position = joint_state.values()
        joint_constraint = Constraint()
        joint_constraint.type = 'JointPositionList'
        joint_constraint.parameter_value_pair = {'goal_state': goal_state}
        return joint_constraint

    def get_current_joint_state_constraints(self, arms: dict) -> Constraint:
        return self.get_simple_joint_constraint(arms)

    ### collision ###
    def get_avoid_all_collision(self, minimum_distance: avoid_collisions_distance) -> CollisionEntry:
        avoid_collision = CollisionEntry()
        avoid_collision.distance = minimum_distance
        avoid_collision.type = CollisionEntry.AVOID_COLLISION
        return avoid_collision

        ###  utils ###

    def _get_point_stamped(self, goal_pose: PoseStamped):
        goal_point = PointStamped()
        goal_point.header = goal_pose.header
        goal_point.point = goal_pose.pose.position
        return goal_point


class BaseGoal(GiskardGoal):

    def __init__(self, goal_pose: PoseStamped, base_velocity_type: str, left_arm_joints: list,
                 right_arm_joints: Optional[list]):
        self.base_convergence_delta_xy: float = 0.05
        self.base_convergence_delta_theta: float = 0.1
        self.base_collision_avoidance_distance: float = 0.2
        self.base_collision_avoidance_hint_vector: Vector3 = Vector3(0, -1, 0)
        self.base_collision_avoidance_hint_link: str = "kitchen_island"
        self.base_max_velocity_fast_xy: float = 0.25
        self.base_max_velocity_fast_theta: float = 0.4
        self.base_max_velocity_slow_xy: float = 0.04
        self.base_max_velocity_slow_theta: float = 0.07
        self.goal_pose = goal_pose
        self.velocity_type = base_velocity_type
        self.left_arm_joints = left_arm_joints
        self.right_arm_joints = right_arm_joints
        self.goal = MoveCmd()

    def make_goal(self):
        self.goal.constraints.append(self.get_cartesian_constraint(root_frame=robot_description.i.odom_frame,
                                                                   tip_frame=robot_description.i.base_frame,
                                                                   goal_pose=self.goal_pose,
                                                                   max_velocity=self.base_max_velocity_fast_xy,
                                                                   avoid_collisions=True))

        avoidance_hint_vector_stamped = Vector3Stamped()
        avoidance_hint_vector_stamped.header.frame_id = self.world  ### fixed_frame
        avoidance_hint_vector_stamped.header.stamp = 0.0
        avoidance_hint_vector_stamped.vector = self.base_collision_avoidance_hint_vector

        self.goal.constraints.append(
            self.get_collision_avoidance_hint_constraint(environment_link=self.base_collision_avoidance_hint_link,
                                                         avoidance_hint=avoidance_hint_vector_stamped, threshold=None, spring_offset=None,
                                                         max_velocity=None))

        if self.velocity_type == 'slow':
            self.goal.constraints.append(
                self.get_base_velocity_constraint(self.base_max_velocity_slow_xy, self.base_max_velocity_slow_theta))
        else:
            self.goal.constraints.append(
                self.get_base_velocity_constraint(self.base_max_velocity_fast_xy, self.base_max_velocity_fast_theta))

        pointing_goal = PoseStamped()
        pointing_goal.header.frame_id = robot_description.i.base_frame
        pointing_goal.header.stamp = 0.0
        pointing_goal.pose.position = Vector3(1, 0, 0)
        pointing_goal.pose.orientation = Quaternion(0, 0, 0, 1)
        self.goal.constraints.append(self.get_head_pointing_constraint(pointing_goal))

        arms = {}
        for joint in self.left_arm_joints:
            arms[joint] = BulletWorld.robot.get_joint_state(joint_name=joint)

        if self.right_arm_joints:
            for joint in self.right_arm_joints:
                arms[joint] = BulletWorld.robot.get_joint_state(joint_name=joint)

        self.goal.constraints.append(self.get_current_joint_state_constraints(arms))

        self.goal.collisions.append(self.get_avoid_all_collision(minimum_distance=self.base_collision_avoidance_distance))

    def send_base_goal(self):
        print(len(self.goal.constraints))
        print(len(self.goal.collisions))
        self.send_goal(self.goal)


