#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

from tf2_ros import Buffer
from tf2_ros import TransformListener

import cv2
import numpy as np


# ==============================================================
# CONSTANTS
# ==============================================================

# Work platform top = 0.05 m
# Cube height = 0.04 m
#
# Cube center:
# 0.05 + 0.02 = 0.07 m
TARGET_Z = 0.07


# ==============================================================
# RED CUBE WORLD DETECTOR
# ==============================================================

class RedCubeWorldDetector(Node):

    def __init__(self):

        super().__init__('red_cube_world_detector')

        # ------------------------------------------------------
        # OpenCV bridge
        # ------------------------------------------------------

        self.bridge = CvBridge()

        # ------------------------------------------------------
        # Camera intrinsics
        # ------------------------------------------------------

        self.fx = None
        self.fy = None

        self.cx_camera = None
        self.cy_camera = None

        # ------------------------------------------------------
        # TF
        # ------------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ------------------------------------------------------
        # Camera image subscriber
        # ------------------------------------------------------

        self.image_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )

        # ------------------------------------------------------
        # Camera info subscriber
        # ------------------------------------------------------

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )

        # ------------------------------------------------------
        # Cube world position publisher
        # ------------------------------------------------------

        self.point_pub = self.create_publisher(
            PointStamped,
            '/red_cube/world_point',
            10
        )

        # Used to reduce terminal printing
        self.frame_count = 0

        self.get_logger().info(
            'Red cube world detector started'
        )


    # ==========================================================
    # CAMERA INFO CALLBACK
    # ==========================================================

    def camera_info_callback(self, msg):

        # Only read the calibration once
        if self.fx is not None:
            return

        # Camera matrix:
        #
        # [ fx  0  cx ]
        # [ 0  fy  cy ]
        # [ 0   0   1 ]

        self.fx = msg.k[0]
        self.fy = msg.k[4]

        self.cx_camera = msg.k[2]
        self.cy_camera = msg.k[5]

        self.get_logger().info(
            f'Camera intrinsics received: '
            f'fx={self.fx:.2f}, '
            f'fy={self.fy:.2f}, '
            f'cx={self.cx_camera:.2f}, '
            f'cy={self.cy_camera:.2f}'
        )


    # ==========================================================
    # QUATERNION -> ROTATION MATRIX
    # ==========================================================

    def quaternion_to_matrix(self, x, y, z, w):

        return np.array([

            [
                1 - 2 * (y*y + z*z),
                2 * (x*y - z*w),
                2 * (x*z + y*w)
            ],

            [
                2 * (x*y + z*w),
                1 - 2 * (x*x + z*z),
                2 * (y*z - x*w)
            ],

            [
                2 * (x*z - y*w),
                2 * (y*z + x*w),
                1 - 2 * (x*x + y*y)
            ]

        ])


    # ==========================================================
    # IMAGE CALLBACK
    # ==========================================================

    def image_callback(self, msg):

        # Wait until /camera_info has been received
        if self.fx is None:
            return


        # ------------------------------------------------------
        # ROS Image -> OpenCV
        # ------------------------------------------------------

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )


        # ------------------------------------------------------
        # BGR -> HSV
        # ------------------------------------------------------

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )


        # ------------------------------------------------------
        # RED HSV RANGES
        #
        # Red wraps around the HSV hue scale.
        #
        # Range 1:
        # H = 0 -> 10
        #
        # Range 2:
        # H = 170 -> 179
        # ------------------------------------------------------

        lower_red_1 = np.array([
            0,
            100,
            100
        ])

        upper_red_1 = np.array([
            10,
            255,
            255
        ])


        lower_red_2 = np.array([
            170,
            100,
            100
        ])

        upper_red_2 = np.array([
            179,
            255,
            255
        ])


        # ------------------------------------------------------
        # Create red masks
        # ------------------------------------------------------

        mask1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2
        )

        mask = cv2.bitwise_or(
            mask1,
            mask2
        )


        # ------------------------------------------------------
        # FIND CONTOURS
        # ------------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        # ------------------------------------------------------
        # NO RED OBJECT FOUND
        # ------------------------------------------------------

        if not contours:

            cv2.imshow(
                'SO101 Red Cube World Position',
                frame
            )

            cv2.waitKey(1)

            return


        # ------------------------------------------------------
        # Largest red contour
        # ------------------------------------------------------

        contour = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(
            contour
        )


        # Ignore very small red noise
        if area < 500:

            cv2.imshow(
                'SO101 Red Cube World Position',
                frame
            )

            cv2.waitKey(1)

            return


        # ------------------------------------------------------
        # BOUNDING BOX
        # ------------------------------------------------------

        x, y, w, h = cv2.boundingRect(
            contour
        )


        # ------------------------------------------------------
        # CENTER PIXEL
        # ------------------------------------------------------

        u = x + w // 2
        v = y + h // 2


        # ------------------------------------------------------
        # PIXEL -> CAMERA RAY
        #
        # X = (u - cx) / fx
        # Y = (v - cy) / fy
        # Z = 1
        # ------------------------------------------------------

        ray_camera = np.array([

            (u - self.cx_camera) / self.fx,

            (v - self.cy_camera) / self.fy,

            1.0

        ])


        # ------------------------------------------------------
        # GET CAMERA OPTICAL FRAME IN WORLD
        # ------------------------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                'world',
                'camera_optical_frame',
                Time()
            )

        except Exception:
            return


        # ------------------------------------------------------
        # CAMERA POSITION IN WORLD
        # ------------------------------------------------------

        camera_position = np.array([

            transform.transform.translation.x,

            transform.transform.translation.y,

            transform.transform.translation.z

        ])


        # ------------------------------------------------------
        # CAMERA ORIENTATION
        # ------------------------------------------------------

        q = transform.transform.rotation

        rotation_matrix = self.quaternion_to_matrix(
            q.x,
            q.y,
            q.z,
            q.w
        )


        # ------------------------------------------------------
        # CAMERA RAY -> WORLD RAY
        # ------------------------------------------------------

        ray_world = rotation_matrix @ ray_camera


        # ------------------------------------------------------
        # INTERSECT RAY WITH CUBE CENTER PLANE
        #
        # Ray equation:
        #
        # P = camera_position + t * ray_world
        #
        # We know:
        #
        # Pz = TARGET_Z
        #
        # Therefore:
        #
        # t =
        # (TARGET_Z - camera_z)
        # ----------------------
        #       ray_world_z
        # ------------------------------------------------------

        if abs(ray_world[2]) < 0.000001:
            return


        t = (
            TARGET_Z - camera_position[2]
        ) / ray_world[2]


        # Intersection is behind camera
        if t <= 0:
            return


        # ------------------------------------------------------
        # WORLD POSITION OF CUBE
        # ------------------------------------------------------

        cube_world = (
            camera_position +
            t * ray_world
        )


        world_x = cube_world[0]
        world_y = cube_world[1]
        world_z = cube_world[2]


        # ======================================================
        # PUBLISH RED CUBE WORLD POSITION
        # ======================================================

        point_msg = PointStamped()

        # Use image timestamp
        point_msg.header.stamp = msg.header.stamp

        # Coordinates are expressed in world
        point_msg.header.frame_id = 'world'

        point_msg.point.x = float(world_x)
        point_msg.point.y = float(world_y)
        point_msg.point.z = float(world_z)

        self.point_pub.publish(
            point_msg
        )


        # ------------------------------------------------------
        # DRAW BOUNDING BOX
        # ------------------------------------------------------

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )


        # ------------------------------------------------------
        # DRAW CENTER PIXEL
        # ------------------------------------------------------

        cv2.circle(
            frame,
            (u, v),
            6,
            (255, 0, 0),
            -1
        )


        # ------------------------------------------------------
        # SHOW WORLD POSITION
        # ------------------------------------------------------

        text = (
            f'X={world_x:.3f} '
            f'Y={world_y:.3f} '
            f'Z={world_z:.3f}'
        )

        cv2.putText(
            frame,
            text,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )


        # ------------------------------------------------------
        # PRINT ONCE EVERY 10 FRAMES
        # ------------------------------------------------------

        self.frame_count += 1

        if self.frame_count % 10 == 0:

            print()
            print('----------------------------')
            print('RED CUBE DETECTED')
            print('----------------------------')

            print(
                f'Pixel: '
                f'u={u}, v={v}'
            )

            print(
                f'World X: '
                f'{world_x:.4f} m'
            )

            print(
                f'World Y: '
                f'{world_y:.4f} m'
            )

            print(
                f'World Z: '
                f'{world_z:.4f} m'
            )

            print(
                'Published topic: '
                '/red_cube/world_point'
            )


        # ------------------------------------------------------
        # DISPLAY IMAGE
        # ------------------------------------------------------

        cv2.imshow(
            'SO101 Red Cube World Position',
            frame
        )

        cv2.waitKey(1)


# ==============================================================
# MAIN
# ==============================================================

def main():

    rclpy.init()

    node = RedCubeWorldDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()