#!/usr/bin/env python3

import time
import math
import threading
import subprocess
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Quaternion
from ros_gz_interfaces.msg import Contacts


ARM_JOINTS = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
]

GRIPPER_JOINT = 'gripper'

OBSERVATION_JOINTS = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
]

TOPDOWN_SEED = [
    0.48204,
    0.06485,
    0.15494,
    1.23061,
    0.38066,
]

OPEN_GRIPPER = 1.5
CLOSED_GRIPPER = 0.331

TARGET_CUBE_Z = 0.07
APPROACH_HEIGHT = 0.08

# Current simulation calibration from the successful top-down grasp.
# cube center -> gripper_frame_link target
GRASP_OFFSET_X = -0.024
GRASP_OFFSET_Y = 0.002
GRASP_OFFSET_Z = 0.010

# Fixed place location for cube center.
PLACE_CUBE_X = 0.15
PLACE_CUBE_Y = 0.10
PLACE_CUBE_Z = 0.07

ARM_MOVE_TIME = 3.0
GRIPPER_MOVE_TIME = 1.5

MIN_CONTOUR_AREA = 500.0
NUM_VISION_SAMPLES = 10


class AutoPickPlace(Node):

    def __init__(self):
        super().__init__('auto_pick_place')

        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_joint_state = None
        self.detections = deque(maxlen=30)
        self.detection_lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

            
        self.red_block_contact = False
        self.moving_jaw_touch = False
        self.fixed_jaw_touch = False
        self.grasp_event = threading.Event()


        self.contact_subscription = self.create_subscription(
            Contacts,
            '/red_block/contacts',
            self.contact_callback,
            10
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
        )

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def camera_info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}'
            )

    def contact_callback(self, msg):

        for contact in msg.contacts:

            collision1 = contact.collision1.name
            collision2 = contact.collision2.name

            # Ignore contact that does not involve red block
            if (
                'red_block' not in collision1
                and 'red_block' not in collision2
            ):
                continue

            # Red block touching moving jaw
            if (
                'moving_jaw_link' in collision1
                or 'moving_jaw_link' in collision2
            ):
                self.moving_jaw_touch = True

            # Red block touching fixed gripper jaw
            if (
                'gripper_link::gripper_link_collision' in collision1
                or 'gripper_link::gripper_link_collision' in collision2
            ):
                self.fixed_jaw_touch = True

            # Successful grasp only after BOTH have been detected
            if self.moving_jaw_touch and self.fixed_jaw_touch:

                if not self.red_block_contact:
                    self.get_logger().info(
                        'REAL GRASP VERIFIED: cube touching both gripper jaws!'
                    )

                self.red_block_contact = True
                self.grasp_event.set()

    def image_callback(self, msg):
        if self.fx is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array([0, 100, 100])
        upper_red_1 = np.array([10, 255, 255])

        lower_red_2 = np.array([170, 100, 100])
        upper_red_2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return

        contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
            return

        x, y, w, h = cv2.boundingRect(contour)

        u = x + w / 2.0
        v = y + h / 2.0

        ray_camera = np.array([
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            1.0,
        ], dtype=float)

        try:
            tf = self.tf_buffer.lookup_transform(
                'world',
                'camera_optical_frame',
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return

        camera_position = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=float)

        q = tf.transform.rotation
        rotation_matrix = self.quaternion_to_matrix(
            q.x, q.y, q.z, q.w
        )

        ray_world = rotation_matrix @ ray_camera

        if abs(ray_world[2]) < 1e-6:
            return

        t = (
            TARGET_CUBE_Z - camera_position[2]
        ) / ray_world[2]

        if t <= 0.0:
            return

        cube_world = camera_position + t * ray_world

        with self.detection_lock:
            self.detections.append(cube_world)

    @staticmethod
    def quaternion_to_matrix(x, y, z, w):
        norm = math.sqrt(x*x + y*y + z*z + w*w)

        if norm == 0.0:
            return np.eye(3)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return np.array([
            [
                1 - 2*(y*y + z*z),
                2*(x*y - z*w),
                2*(x*z + y*w),
            ],
            [
                2*(x*y + z*w),
                1 - 2*(x*x + z*z),
                2*(y*z - x*w),
            ],
            [
                2*(x*z - y*w),
                2*(y*z + x*w),
                1 - 2*(x*x + y*y),
            ],
        ], dtype=float)

    @staticmethod
    def wait_future(future, timeout_sec=10.0):
        start = time.time()

        while not future.done():
            if time.time() - start > timeout_sec:
                raise TimeoutError('ROS future timed out')
            time.sleep(0.02)

        return future.result()

    def wait_for_interfaces(self):
        self.get_logger().info('Waiting for arm controller...')
        while not self.arm_client.wait_for_server(timeout_sec=1.0):
            pass

        self.get_logger().info('Waiting for gripper controller...')
        while not self.gripper_client.wait_for_server(timeout_sec=1.0):
            pass

        self.get_logger().info('Waiting for /compute_ik...')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            pass

        self.get_logger().info('Waiting for /joint_states...')
        while self.latest_joint_state is None:
            time.sleep(0.1)

        self.get_logger().info('Waiting for /camera_info...')
        while self.fx is None:
            time.sleep(0.1)

    def send_trajectory(
        self,
        action_client,
        joint_names,
        positions,
        duration_sec,
    ):
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)

        point.time_from_start = DurationMsg(
            sec=sec,
            nanosec=nanosec,
        )

        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = action_client.send_goal_async(goal)
        goal_handle = self.wait_future(send_future, 5.0)

        if not goal_handle.accepted:
            raise RuntimeError('Trajectory goal was rejected')

        result_future = goal_handle.get_result_async()
        self.wait_future(
            result_future,
             120.0,
        )

    def move_arm(self, joint_positions, duration=ARM_MOVE_TIME):
        self.get_logger().info(
            'Moving arm to: '
            + str([round(v, 4) for v in joint_positions])
        )

        self.send_trajectory(
            self.arm_client,
            ARM_JOINTS,
            joint_positions,
            duration,
        )

    def get_gripper_position(self):

        if self.latest_joint_state is None:
            return None

        try:
            index = self.latest_joint_state.name.index(GRIPPER_JOINT)
        except ValueError:
            return None

        if index >= len(self.latest_joint_state.position):
            return None

        return float(self.latest_joint_state.position[index])


    def wait_for_gripper_position(
        self,
        target,
        tolerance=0.005,
        timeout=300.0,
    ):

        self.get_logger().info(
            f'Waiting for actual gripper position near {target:.3f}...'
        )

        start = time.time()
        last_log = 0.0

        while rclpy.ok():

            position = self.get_gripper_position()

            if position is not None:

                error = abs(position - target)

                if time.time() - last_log > 2.0:
                    self.get_logger().info(
                        f'Gripper actual={position:.4f}, '
                        f'target={target:.4f}'
                    )
                    last_log = time.time()

                # Normal case:
                # gripper reached requested position
                if error <= tolerance:

                    self.get_logger().info(
                        f'Gripper reached target: {position:.4f}'
                    )

                    return True


                # GRASP CASE:
                # Cube physically prevents the gripper from
                # reaching exactly 0.331.
                #
                # If both jaws are touching the cube and the
                # gripper has closed to about 0.34, this means
                # the cube is between the fingers.
                if (
                    abs(target - CLOSED_GRIPPER) < 0.001
                    and position <= 0.340
                    and self.grasp_event.is_set()
                ):

                    self.get_logger().info(
                        f'Gripper stopped on cube at {position:.4f}. '
                        'Both jaws have contact.'
                    )

                    return True


            if time.time() - start > timeout:

                self.get_logger().warning(
                    'Timed out waiting for gripper position.'
                )

                return False

            time.sleep(0.05)

        return False

    def move_gripper(self, position):

        self.get_logger().info(
            f'Moving gripper to {position:.3f}'
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = [GRIPPER_JOINT]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]

        # Give controller 2 SIMULATION seconds to perform motion.
        point.time_from_start = DurationMsg(sec=2)

        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.gripper_client.send_goal_async(goal)

        goal_handle = self.wait_future(
            future,
            5.0,
        )

        if not goal_handle.accepted:
            raise RuntimeError(
                'Gripper goal rejected'
            )

        # IMPORTANT:
        # Do NOT use time.sleep(5) anymore.
        #
        # Wait until /joint_states tells us the gripper
        # really reached its target.
        reached = self.wait_for_gripper_position(
            position,
            tolerance=0.005,
            timeout=300.0,
        )

        if not reached:
            raise RuntimeError(
                f'Gripper did not reach target {position:.3f}'
            )


    def get_gripper_orientation(self):
        tf = self.tf_buffer.lookup_transform(
            'world',
            'gripper_frame_link',
            Time(),
            timeout=Duration(seconds=2.0),
        )

        q = tf.transform.rotation

        self.get_logger().info(
            'Locked top-down gripper orientation: '
            f'[{q.x:.4f}, {q.y:.4f}, '
            f'{q.z:.4f}, {q.w:.4f}]'
        )

        return q

    def compute_ik(self, x, y, z, orientation, seed=None):
        request = GetPositionIK.Request()

        request.ik_request.group_name = 'arm'
        request.ik_request.ik_link_name = 'gripper_frame_link'

        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose = request.ik_request.pose_stamped.pose

        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        pose.orientation.x = orientation.x
        pose.orientation.y = orientation.y
        pose.orientation.z = orientation.z
        pose.orientation.w = orientation.w

        request.ik_request.timeout.sec = 2
        request.ik_request.avoid_collisions = False

        if seed is None:
            seed=TOPDOWN_SEED

        request.ik_request.robot_state.joint_state.name = ARM_JOINTS
        request.ik_request.robot_state.joint_state.position = seed
        request.ik_request.robot_state.is_diff = True

       

        future = self.ik_client.call_async(request)
        response = self.wait_future(future, 5.0)

        if response.error_code.val != 1:
            raise RuntimeError(
                f'IK failed. MoveIt error code: '
                f'{response.error_code.val}'
            )

        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position
        solution = dict(zip(names, positions))

        missing = [
            name for name in ARM_JOINTS
            if name not in solution
        ]

        if missing:
            raise RuntimeError(
                f'IK solution missing joints: {missing}'
            )

        arm_solution = [
            solution[name]
            for name in ARM_JOINTS
        ]

        self.get_logger().info(
            'IK solution: '
            + str([round(v, 4) for v in arm_solution])
        )

        return arm_solution

    def get_cube_position(self):
        with self.detection_lock:
            self.detections.clear()

        self.get_logger().info('Looking for the red cube...')

        start = time.time()

        while True:
            with self.detection_lock:
                count = len(self.detections)

                if count >= NUM_VISION_SAMPLES:
                    samples = np.array(
                        list(self.detections)[-NUM_VISION_SAMPLES:]
                    )
                    break

            if time.time() - start > 10.0:
                raise RuntimeError(
                    'Could not get enough red-cube detections'
                )

            time.sleep(0.1)

        cube = np.median(samples, axis=0)

        self.get_logger().info(
            f'Cube world position: '
            f'X={cube[0]:.4f}, '
            f'Y={cube[1]:.4f}, '
            f'Z={cube[2]:.4f}'
        )

        return cube

    def gazebo_attach(self):
        self.get_logger().info('Attaching cube in Gazebo...')

        subprocess.run(
            [
                'ign',
                'topic',
                '-t',
                '/red_block/attach',
                '-m',
                'ignition.msgs.Empty',
                '-p',
                ' ',
            ],
            check=False,
        )

    def gazebo_detach(self):
        self.get_logger().info('Detaching cube in Gazebo...')

        subprocess.run(
            [
                'ign',
                'topic',
                '-t',
                '/red_block/detach',
                '-m',
                'ignition.msgs.Empty',
                '-p',
                ' ',
            ],
            check=False,
        )

    @staticmethod
    def grasp_frame_target(cube_x, cube_y, cube_z):
        x = cube_x + GRASP_OFFSET_X
        y = cube_y + GRASP_OFFSET_Y
        z = cube_z + GRASP_OFFSET_Z

        return x, y, z

    def run(self):
        self.wait_for_interfaces()

        # Plugin may start attached.
        self.gazebo_detach()
        time.sleep(0.5)

        # 1. Open
        self.move_gripper(OPEN_GRIPPER)

        # 2. Go to known top-down observation pose
        self.move_arm(OBSERVATION_JOINTS)
        time.sleep(1.0)

        # Capture the orientation from this proven pose
        topdown_orientation = Quaternion()

        topdown_orientation.x = 0.9119
        topdown_orientation.y = -0.4059
        topdown_orientation.z = 0.0604
        topdown_orientation.w = 0.0012

        # 3. Camera -> cube world XYZ
        cube_x, cube_y, cube_z = self.get_cube_position()

        # 4. Pick targets
        pick_x, pick_y, pick_grasp_z = self.grasp_frame_target(
            cube_x,
            cube_y,
            cube_z,
        )

        pick_approach_z = pick_grasp_z + APPROACH_HEIGHT

        self.get_logger().info(
            f'Pick approach: '
            f'X={pick_x:.4f}, Y={pick_y:.4f}, '
            f'Z={pick_approach_z:.4f}'
        )

        self.get_logger().info(
            f'Pick grasp: '
            f'X={pick_x:.4f}, Y={pick_y:.4f}, '
            f'Z={pick_grasp_z:.4f}'
        )

        # 5. Move 8 cm above the grasp point
        pick_approach_joints = self.compute_ik(
            pick_x,
            pick_y,
            pick_approach_z,
            topdown_orientation,
        )
        self.move_arm(pick_approach_joints)

        # 6. Descend exactly 8 cm
        pick_grasp_joints = self.compute_ik(
            pick_x,
            pick_y,
            pick_grasp_z,
            topdown_orientation,
            seed=pick_approach_joints,
        )
        self.move_arm(pick_grasp_joints)

        # 7. Close gripper first
        # Reset contact before attempting grasp
        # Clear old contact BEFORE closing
        self.red_block_contact = False
        self.moving_jaw_touch = False
        self.fixed_jaw_touch = False
        self.grasp_event.clear()

        # Close gripper
        self.move_gripper(CLOSED_GRIPPER)

        # Clear contacts that happened during closing
        self.red_block_contact = False
        self.moving_jaw_touch = False
        self.fixed_jaw_touch = False
        self.grasp_event.clear()

        # Real physical grasp
        
        self.get_logger().info(
            'Waiting for grasp contact verification...'
        )

        grasp_detected = self.grasp_event.wait(timeout=8.0)

        if grasp_detected:

            self.get_logger().info(
                'Grasp verified by contact sensor. Attaching cube.'
            )

            self.gazebo_attach()
            time.sleep(0.5)

            # Relax physical contact after attaching
            self.move_gripper(0.35)

        else:

            self.get_logger().warning(
                'GRASP FAILED: no contact with red block. Cube will NOT be attached.'
            )

            return

        # 9. Lift
        self.move_arm(pick_approach_joints)

        # 10. Place targets
        place_x, place_y, place_grasp_z = self.grasp_frame_target(
            PLACE_CUBE_X,
            PLACE_CUBE_Y,
            PLACE_CUBE_Z,
        )

        place_approach_z = (
            place_grasp_z + APPROACH_HEIGHT
        )

        # 11. Move above place location
        place_approach_joints = self.compute_ik(
            place_x,
            place_y,
            place_approach_z,
            topdown_orientation,
        )
        self.move_arm(place_approach_joints)

        # 12. Descend
        place_grasp_joints = self.compute_ik(
            place_x,
            place_y,
            place_grasp_z,
            topdown_orientation,
        )
        self.move_arm(place_grasp_joints)

     # 13. Open gripper FIRST while cube is still attached
        self.move_gripper(OPEN_GRIPPER)
        time.sleep(0.5)

    # 14. Detach cube after the fingers are clear
        self.gazebo_detach()
        time.sleep(0.5)

        # 15. Retreat upward
        # 15. Calculate a new retreat from the CURRENT grasp configuration
        place_retreat_joints = self.compute_ik(
            place_x,
            place_y,
            place_approach_z,
            topdown_orientation,
            seed=place_grasp_joints,
        )

        self.move_arm(place_retreat_joints)

        self.get_logger().info(
            'PICK AND PLACE SEQUENCE FINISHED.'
        )


def main(args=None):
    rclpy.init(args=args)

    node = AutoPickPlace()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    spin_thread.start()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        node.get_logger().error(
            f'Pick-and-place failed: {exc}'
        )

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
