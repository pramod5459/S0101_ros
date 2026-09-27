# SO-101 Autonomous Vision-Based Pick-and-Place

A ROS 2 manipulation project for the **SO-101 5-DOF robotic arm** that performs autonomous vision-guided pick-and-place in Gazebo.

The system detects a red cube using an RGB camera, estimates its position in the world frame, computes inverse kinematics with MoveIt 2, controls the robot using `ros2_control`, verifies the grasp using contact sensing and gripper joint feedback, and transports the cube to a target location.

---

## Demo

> Demo video / GIF coming soon.

### Autonomous pipeline

```text
RGB Camera
    |
    v
OpenCV Red Object Detection
    |
    v
Pixel Coordinates (u, v)
    |
    v
Camera Ray Projection
    |
    v
TF2: camera_optical_frame -> world
    |
    v
Cube Position (X, Y, Z)
    |
    v
MoveIt 2 Inverse Kinematics
    |
    v
ros2_control Joint Trajectories
    |
    v
Physical Grasp
    |
    v
Two-Jaw Contact Verification
    |
    v
Pick -> Lift -> Transport -> Place
```

---

## Features

- ROS 2 Humble
- Gazebo / Ignition Fortress simulation
- SO-101 URDF/Xacro robot model
- `ros2_control`
- Joint trajectory controllers
- MoveIt 2 inverse kinematics
- OpenCV color-based object detection
- Camera intrinsic parameter usage
- Pixel-to-world coordinate estimation
- TF2 coordinate transformations
- Vision-guided grasp positioning
- Gripper position feedback from `/joint_states`
- Gazebo contact sensing
- Two-jaw grasp verification
- Automated pick-and-place sequence

---

## System Architecture

The project combines perception, coordinate transformation, kinematics, robot control and physical grasp verification.

```text
               +---------------------+
               |      RGB Camera     |
               +----------+----------+
                          |
                          v
               +---------------------+
               |       OpenCV        |
               |  Red Cube Detection |
               +----------+----------+
                          |
                          v
               +---------------------+
               | Camera Projection   |
               | + TF2 Transform     |
               +----------+----------+
                          |
                          v
               +---------------------+
               | Cube World Position |
               |     X, Y, Z         |
               +----------+----------+
                          |
                          v
               +---------------------+
               |      MoveIt 2       |
               |  Inverse Kinematics |
               +----------+----------+
                          |
                          v
               +---------------------+
               |    ros2_control     |
               | Joint Trajectories  |
               +----------+----------+
                          |
                          v
               +---------------------+
               |   Contact Sensor    |
               | + Joint Feedback    |
               +----------+----------+
                          |
                          v
               +---------------------+
               | Pick and Place      |
               +---------------------+
```

---

## Perception

The simulated RGB camera publishes:

```text
/camera
/camera_info
```

The vision pipeline:

1. Receives the RGB image.
2. Converts the image from BGR to HSV.
3. Creates masks for red pixels.
4. Finds contours.
5. Selects the largest valid contour.
6. Calculates the center pixel of the detected cube.

```text
Image
  |
  v
HSV Conversion
  |
  v
Red Color Mask
  |
  v
Contour Detection
  |
  v
Cube Pixel Center (u, v)
```

Multiple measurements are collected and the median is used to improve position stability.

---

## Camera Pixel to World Coordinates

The detected image pixel is converted into a 3D camera ray using the camera intrinsic parameters:

```text
x = (u - cx) / fx
y = (v - cy) / fy
z = 1
```

TF2 provides the transformation between:

```text
camera_optical_frame
        |
        v
      world
```

The camera ray is transformed into the world coordinate frame.

The ray is then intersected with the known cube-height plane to estimate:

```text
Cube Position = [X, Y, Z]
```

Example output:

```text
Cube world position: X=0.2483, Y=-0.1016, Z=0.0700
```

---

## Motion Planning

MoveIt 2 is used for inverse kinematics.

The robot calculates joint configurations for:

```text
Observation
    |
    v
Pick Approach
    |
    v
Pick Grasp
    |
    v
Lift
    |
    v
Place Approach
    |
    v
Place
    |
    v
Retreat
```

The project uses:

```text
KDLKinematicsPlugin
```

with position-only IK for the 5-DOF arm.

---

## Robot Control

The robot is controlled using `ros2_control`.

### Arm joints

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
```

### Gripper joint

```text
gripper
```

The arm and gripper use:

```text
JointTrajectoryController
```

Joint states are monitored through:

```text
/joint_states
```

---

## Grasp Verification

A major goal of this project was to avoid assuming that a grasp succeeded simply because a gripper close command was sent.

The system verifies the grasp using physical simulation feedback.

### Contact Sensor

The cube contains a Gazebo contact sensor.

A successful grasp requires contact with both sides of the gripper:

```text
moving_jaw_link
```

and

```text
gripper_link
```

Only after both contacts are detected does the program consider the cube grasped.

---

## Gripper Joint Feedback

The commanded closed gripper position is approximately:

```text
0.331
```

When the cube is physically between the fingers, the cube can prevent the gripper from reaching the exact target.

Example from a successful run:

```text
Target: 0.331
Actual: 0.3398
```

The program checks:

```text
gripper position
        +
two-jaw contact
```

If both jaws are contacting the cube and the gripper is physically blocked near the closed position, the grasp is accepted.

Example:

```text
REAL GRASP VERIFIED: cube touching both gripper jaws!

Gripper stopped on cube at 0.3398.
Both jaws have contact.
```

This prevents the program from waiting indefinitely for an unreachable joint target.

---

## Grasp Validation Sequence

The grasp logic follows this sequence:

```text
Clear previous contact state
        |
        v
Close gripper
        |
        v
Detect cube blocking gripper
        |
        v
Verify both-jaw contact
        |
        v
Clear temporary closing contacts
        |
        v
Require fresh contact verification
        |
        v
Accept grasp
```

The cube is only attached in simulation **after the physical grasp has been verified**.

---

## Autonomous Pick-and-Place Sequence

```text
1. Open gripper

2. Move to observation pose

3. Detect red cube using camera

4. Estimate cube world coordinates

5. Calculate pick approach IK

6. Move above cube

7. Calculate grasp IK

8. Descend to cube

9. Close gripper

10. Verify two-jaw contact

11. Verify gripper position feedback

12. Attach cube after verified grasp

13. Relax gripper contact pressure

14. Lift cube

15. Move to place approach

16. Descend to place position

17. Open gripper

18. Detach cube

19. Retreat upward
```

---

## Repository Structure

```text
S0101_ros/
│
├── so101_description/
│   ├── launch/
│   ├── meshes/
│   ├── models/
│   │   ├── red_block/
│   │   └── work_platform/
│   ├── rviz/
│   ├── urdf/
│   └── worlds/
│
├── so101_controller/
│   ├── config/
│   └── launch/
│
├── so101_moveit_config/
│   ├── config/
│   └── launch/
│
└── so101_bringup/
    ├── launch/
    │   └── simulated_robot.launch.py
    │
    └── scripts/
        ├── auto_pick_place.py
        ├── red_cube_world.py
        ├── pregrasp_from_vision.py
        └── descend_to_cube.py
```

---

## Main Autonomous Node

The complete autonomous pipeline is implemented in:

```text
so101_bringup/scripts/auto_pick_place.py
```

It integrates:

```text
OpenCV
TF2
MoveIt 2
ros2_control
Gazebo contact sensing
Joint-state feedback
```

---

## Technologies Used

| Technology | Purpose |
|---|---|
| ROS 2 Humble | Robotics middleware |
| Gazebo / Ignition Fortress | Robot simulation |
| MoveIt 2 | Inverse kinematics |
| ros2_control | Robot joint control |
| OpenCV | Object detection |
| TF2 | Coordinate transformations |
| Python | Autonomous manipulation logic |
| URDF / Xacro | Robot modeling |
| ros_gz_bridge | Gazebo-to-ROS communication |

---

## Requirements

Tested with:

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo / Ignition Fortress
MoveIt 2
ros2_control
ros_gz
OpenCV
Python 3
```

---

## Installation

Create a workspace:

```bash
mkdir -p ~/so101_ws/src
cd ~/so101_ws/src
```

Clone the repository:

```bash
git clone git@github.com:pramod5459/S0101_ros.git
```

Build:

```bash
cd ~/so101_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

# Running the Project

The system currently uses separate terminals for simulation, MoveIt, bridges and the autonomous node.

---

## Terminal 1 — Gazebo + SO-101 + Controllers

```bash
cd ~/so101_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch so101_bringup simulated_robot.launch.py
```

---

## Terminal 2 — MoveIt 2

```bash
cd ~/so101_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch so101_moveit_config move_group.launch.py
```

Set MoveIt to simulation time:

```bash
ros2 param set /move_group use_sim_time true
```

---

## Terminal 3 — Camera Bridge

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
/camera@sensor_msgs/msg/Image[ignition.msgs.Image \
/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo
```

---

## Terminal 4 — Contact Sensor Bridge

```bash
source /opt/ros/humble/setup.bash
source ~/so101_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
'/red_block/contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
```

---

## Terminal 5 — Autonomous Pick-and-Place

```bash
cd ~/so101_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/S0101_ros/so101_bringup/scripts/auto_pick_place.py
```

---

## Example Successful Output

```text
Cube world position:
X=0.2483
Y=-0.1016
Z=0.0700

REAL GRASP VERIFIED:
cube touching both gripper jaws!

Gripper stopped on cube at 0.3398.
Both jaws have contact.

Grasp verified by contact sensor.
Attaching cube.

PICK AND PLACE SEQUENCE FINISHED.
```

---

## Important ROS Interfaces

### Topics

```text
/camera
/camera_info
/joint_states
/red_block/contacts
```

### Actions

```text
/arm_controller/follow_joint_trajectory

/gripper_controller/follow_joint_trajectory
```

### Service

```text
/compute_ik
```

---

## Technical Concepts Demonstrated

This project demonstrates practical experience with:

- ROS 2 nodes
- Publishers and subscribers
- ROS 2 actions
- ROS 2 services
- URDF and Xacro
- TF2 coordinate frames
- Camera projection geometry
- Computer vision with OpenCV
- Inverse kinematics
- MoveIt 2
- `ros2_control`
- Joint trajectory control
- Sensor feedback
- Gazebo contact sensors
- Event-driven grasp verification
- Autonomous robotic manipulation

---

## Current Status

The simulated SO-101 arm can autonomously:

```text
Detect Object
     |
     v
Estimate Position
     |
     v
Calculate IK
     |
     v
Approach
     |
     v
Grasp
     |
     v
Verify Physical Contact
     |
     v
Lift
     |
     v
Transport
     |
     v
Place
```

The current calibrated cube position has completed multiple consecutive autonomous pick-and-place runs successfully.

---

## Future Work

Planned development includes:

- Test grasping throughout a larger workspace
- Multiple-object detection
- Improved grasp pose estimation
- Collision-aware motion planning
- Real SO-101 hardware integration
- Real camera calibration
- Hardware vision-based pick-and-place
- Additional object classes
- Imitation learning
- Reinforcement learning
- Vision-language-action experiments

---

## Author

**Pramod Panta**

Mechanical Engineering background with interests in robotics, autonomous systems, robot perception, manipulation and robot learning.

GitHub: [pramod5459](https://github.com/pramod5459)
