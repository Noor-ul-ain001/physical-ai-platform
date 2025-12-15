---
sidebar_position: 4
---

# Week 4: TF and URDF

## Overview

This week covers two critical concepts in robotics: TF (Transforms) for representing spatial relationships, and URDF (Unified Robot Description Format) for describing robot structure and kinematics. These tools are essential for robot perception, navigation, and manipulation.

## Learning Objectives

By the end of this week, you will:
- Understand the concept of coordinate frames and transforms
- Work with the TF2 system in ROS 2
- Create and interpret URDF files
- Use robot state publishers
- Implement frame transformations
- Debug TF trees

## TF (Transforms) in ROS 2

TF (Transforms) is a package that lets the user keep track of multiple coordinate frames over time. It maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc., between any two coordinate frames at any desired point in time.

### Coordinate Frames

A coordinate frame is a reference system that defines position and orientation in space. Common frames in robotics include:
- `map`: World-fixed frame
- `odom`: Odometry-based frame
- `base_link`: Robot's base frame
- `camera_frame`: Camera's reference frame
- `tool0`: Tool end-effector frame

### TF2 Concepts

TF2 is the second generation of the transform library in ROS. It provides:

1. **Transform Storage**: Maintains a tree of coordinate frame transformations
2. **Lookup**: Retrieves transformations between any two frames
3. **Interpolation**: Provides transformations at specific times
4. **Extrapolation Protection**: Prevents use of outdated transforms

### Basic TF2 Usage

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        
        # Create a buffer to store transforms
        self.tf_buffer = Buffer()
        
        # Create a transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to periodically lookup transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

    def lookup_transform(self):
        try:
            # Look up transform from 'base_link' to 'camera_link' at current time
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time())
                
            self.get_logger().info(
                f'Transform: ({t.transform.translation.x}, '
                f'{t.transform.translation.y}, '
                f'{t.transform.translation.z})')
                
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return
```

### Publishing Transforms

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # Define the transform
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
```

## URDF (Unified Robot Description Format)

URDF is an XML format for representing a robot model. It defines the kinematic and dynamic structure of a robot, including links, joints, materials, and visual/inertial properties.

### URDF Components

- **Links**: Rigid parts of the robot (e.g., chassis, wheels, end-effector)
- **Joints**: Connections between links (e.g., revolute, prismatic)
- **Visual**: How the robot appears in simulation/visualization
- **Inertial**: Mass, center of mass, and inertia properties
- **Collision**: Collision detection geometry

### Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0"
               izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel links -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Joints connecting wheels to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="-0.15 0.25 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="-0.15 -0.25 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Robot State Publisher

The robot_state_publisher package takes the joint angles of a robot and calculates the 3D poses of the robot links. It publishes the transforms between the links using the tf2 package.

### Using Robot State Publisher

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        
        # Initialize joint state publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [math.sin(self.get_clock().now().nanoseconds / 1e9),
                        math.cos(self.get_clock().now().nanoseconds / 1e9)]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Publish joint states
        self.joint_pub.publish(msg)
```

## TF and URDF Commands

Useful command-line tools for working with TF and URDF:

```bash
# TF tools
ros2 run tf2_tools view_frames                    # View TF tree
ros2 run tf2_ros tf2_echo frame_id                # Echo transform
ros2 run rviz2 rviz2                              # Visualize TF tree in RViz

# URDF tools
check_urdf /path/to/robot.urdf                    # Validate URDF
ros2 run xacro xacro /path/to/robot.xacro > robot.urdf  # Process Xacro
```

## Practical Example: Mobile Robot URDF

Here's a more complete mobile robot URDF example with TF setup:

```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>

  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0"
               iyy="0.083" iyz="0.0"
               izz="0.133"/>
    </inertial>
  </link>

  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Camera joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Left wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.18 -0.1" rpy="-1.5708 0 0"/>
  </joint>

  <!-- Right wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.18 -0.1" rpy="-1.5708 0 0"/>
  </joint>
</robot>
```

## Common TF Issues and Solutions

1. **Timestamp issues**: Ensure transforms are published with correct timestamps
2. **Buffer timeout**: Increase buffer size or frequency of transform publishing
3. **Circular dependencies**: Avoid loops in the TF tree
4. **Extrapolation errors**: Wait for transforms to be buffered before looking them up

## Integration with Perception and Navigation

TF is critical for:
- Sensor fusion: Transform sensor data to a common frame
- Localization: Transform between map and odom frames
- Navigation: Plan trajectories in the map frame
- Manipulation: Transform between base and end-effector frames

## Summary

This week covered TF (Transforms) and URDF, essential tools for representing spatial relationships and robot structure in ROS 2. You learned how to work with coordinate frames, implement TF2 listeners and broadcasters, create URDF models, and publish joint states. These concepts are fundamental for robot perception, navigation, and manipulation.

## Exercises

1. Create a URDF for a simple robot with at least 3 links and 2 joints
2. Implement a TF broadcaster that publishes transforms for a moving robot
3. Use robot_state_publisher to visualize joint states in RViz
4. Debug a TF tree with a common issue (timestamp errors, buffer timeouts)