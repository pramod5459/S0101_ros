#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_msgs.srv import GetPositionIK

from tf2_ros import Buffer
from tf2_ros import TransformListener


class PreGraspNode(Node):

    def __init__(self):

        super().__init__('descend_to_cube')

        # Only execute once
        self.executed = False

        # Store latest robot joint state
        self.current_joint_state = None

        # ------------------------------------------------------
        # TF
        # ------------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ------------------------------------------------------
        # Subscribe to detected cube position
        # ------------------------------------------------------

        self.cube_sub = self.create_subscription(
            PointStamped,
            '/red_cube/world_point',
            self.cube_callback,
            10
        )

        # ------------------------------------------------------
        # Subscribe to CURRENT robot joints
        # ------------------------------------------------------

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # ------------------------------------------------------
        # MoveIt IK service
        # ------------------------------------------------------

        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        # ------------------------------------------------------
        # Arm controller publisher
        # ------------------------------------------------------

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.get_logger().info(
            'Waiting for joint states and red cube position...'
        )


    # ==========================================================
    # JOINT STATE CALLBACK
    # ==========================================================

    def joint_state_callback(self, msg):

        self.current_joint_state = msg


    # ==========================================================
    # CUBE CALLBACK
    # ==========================================================

    def cube_callback(self, msg):

        if self.executed:
            return

        # ------------------------------------------------------
        # Make sure we know current robot configuration
        # ------------------------------------------------------

        if self.current_joint_state is None:

            self.get_logger().warn(
                'Waiting for /joint_states...'
            )

            return

        cube_x = msg.point.x
        cube_y = msg.point.y
        cube_z = msg.point.z

        self.get_logger().info(
            f'Cube detected: '
            f'X={cube_x:.3f}, '
            f'Y={cube_y:.3f}, '
            f'Z={cube_z:.3f}'
        )

        # ------------------------------------------------------
        # PRE-GRASP TARGET
        #
        # Move 8 cm above cube
        # ------------------------------------------------------

        target_x = cube_x + 0.010
        target_y = cube_y - 0.015
        target_z = cube_z + 0.02

        self.get_logger().info(
            f'Pre-grasp target: '
            f'X={target_x:.3f}, '
            f'Y={target_y:.3f}, '
            f'Z={target_z:.3f}'
        )

        # ------------------------------------------------------
        # Get current end-effector orientation
        # ------------------------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                'world',
                'gripper_frame_link',
                Time()
            )

        except Exception as e:

            self.get_logger().warn(
                f'Waiting for gripper TF: {e}'
            )

            return

        q = transform.transform.rotation

        # ------------------------------------------------------
        # Check IK service
        # ------------------------------------------------------

        if not self.ik_client.service_is_ready():

            self.get_logger().warn(
                '/compute_ik is not ready'
            )

            return

        # ------------------------------------------------------
        # Build IK request
        # ------------------------------------------------------

        request = GetPositionIK.Request()

        request.ik_request.group_name = 'arm'

        request.ik_request.ik_link_name = (
            'gripper_frame_link'
        )

        request.ik_request.pose_stamped.header.frame_id = (
            'world'
        )

        # ------------------------------------------------------
        # IMPORTANT:
        # Give MoveIt CURRENT robot joint state as IK seed
        # ------------------------------------------------------

        request.ik_request.robot_state.joint_state.name = list(
            self.current_joint_state.name
        )

        request.ik_request.robot_state.joint_state.position = list(
            self.current_joint_state.position
        )

        request.ik_request.robot_state.is_diff = False

        # ------------------------------------------------------
        # Position target
        # ------------------------------------------------------

        request.ik_request.pose_stamped.pose.position.x = (
            target_x
        )

        request.ik_request.pose_stamped.pose.position.y = (
            target_y
        )

        request.ik_request.pose_stamped.pose.position.z = (
            target_z
        )

        # ------------------------------------------------------
        # Keep current orientation
        # ------------------------------------------------------

        request.ik_request.pose_stamped.pose.orientation.x = q.x
        request.ik_request.pose_stamped.pose.orientation.y = q.y
        request.ik_request.pose_stamped.pose.orientation.z = q.z
        request.ik_request.pose_stamped.pose.orientation.w = q.w

        # ------------------------------------------------------
        # IK settings
        # ------------------------------------------------------

        request.ik_request.avoid_collisions = True

        request.ik_request.timeout.sec = 3

        self.get_logger().info(
            'Requesting IK solution...'
        )

        future = self.ik_client.call_async(
            request
        )

        future.add_done_callback(
            self.ik_result_callback
        )

        # Prevent repeated requests
        self.executed = True


    # ==========================================================
    # IK RESULT CALLBACK
    # ==========================================================

    def ik_result_callback(self, future):

        try:

            response = future.result()

        except Exception as e:

            self.get_logger().error(
                f'IK service failed: {e}'
            )

            self.executed = False
            return

        # MoveIt success code = 1
        if response.error_code.val != 1:

            self.get_logger().error(
                f'IK failed. '
                f'Error code: {response.error_code.val}'
            )

            self.executed = False
            return

        self.get_logger().info(
            'IK solution found!'
        )

        # ------------------------------------------------------
        # Extract solution joints
        # ------------------------------------------------------

        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position

        joint_map = dict(
            zip(names, positions)
        )

        arm_joints = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll'
        ]

        try:

            arm_positions = [
                joint_map[name]
                for name in arm_joints
            ]

        except KeyError as e:

            self.get_logger().error(
                f'Missing IK joint: {e}'
            )

            self.executed = False
            return

        self.get_logger().info(
            f'IK joints: {arm_positions}'
        )

        # ------------------------------------------------------
        # Build trajectory
        # ------------------------------------------------------

        trajectory = JointTrajectory()

        trajectory.joint_names = arm_joints

        point = JointTrajectoryPoint()

        point.positions = arm_positions

        point.time_from_start.sec = 4

        trajectory.points.append(
            point
        )

        # ------------------------------------------------------
        # Send trajectory to arm controller
        # ------------------------------------------------------

        self.arm_pub.publish(
            trajectory
        )

        self.get_logger().info(
            'Pre-grasp trajectory sent.'
        )


def main():

    rclpy.init()

    node = PreGraspNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()