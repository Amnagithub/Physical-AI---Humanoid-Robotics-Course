---
sidebar_position: 3
---

# Chapter 3: Controlling Humanoid Robots with Python and URDF

## Learning Objectives

After completing this chapter, you should be able to:
- Read and understand URDF (Unified Robot Description Format) files for humanoid robots
- Modify URDF files to change robot configurations
- Interface Python AI agents with ROS controllers for humanoid robots
- Implement basic humanoid robot control using ROS control framework
- Understand the kinematics of humanoid robots
- Work with sensor integration for humanoid robots
- Apply MoveIt! for humanoid motion planning

## Table of Contents
- [Introduction to URDF](#introduction-to-urdf)
- [Humanoid Robot Kinematics](#humanoid-robot-kinematics)
- [Reading and Understanding Humanoid URDF Files](#reading-and-understanding-humanoid-urdf-files)
- [Modifying Humanoid URDF Configurations](#modifying-humanoid-urdf-configurations)
- [ROS Controllers for Humanoid Robots](#ros-controllers-for-humanoid-robots)
- [Python Scripts for Humanoid Control](#python-scripts-for-humanoid-control)
- [MoveIt! Integration for Motion Planning](#moveit-integration-for-motion-planning)
- [Sensor Integration for Humanoid Robots](#sensor-integration-for-humanoid-robots)
- [AI Agent Integration with ROS Controllers](#ai-agent-integration-with-ros-controllers)
- [Practical Example: Simple Humanoid Walking Gait](#practical-example-simple-humanoid-walking-gait)
- [Summary](#summary)

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF files define the physical structure, including links (rigid bodies), joints (connections between links), and other properties like mass, inertia, and visual/physical appearance.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Links represent rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### URDF for Humanoid Robots

Humanoid robots have a specific structure with:
- A torso/upper body
- Two arms with shoulders, elbows, and wrists
- A head
- Two legs with hips, knees, and ankles
- Multiple joints connecting these parts

URDF files for humanoid robots can be quite complex, with dozens of links and joints.

## Humanoid Robot Kinematics

Kinematics is the study of motion without considering the forces that cause it. For humanoid robots, we focus on:

### Forward Kinematics
Given joint angles, calculate the position and orientation of the end effector (e.g., hand or foot).

### Inverse Kinematics
Given a desired position and orientation of the end effector, calculate the required joint angles.

### Kinematic Chains in Humanoid Robots
- Leg chains: hip → knee → ankle
- Arm chains: shoulder → elbow → wrist
- Full-body chains: connecting multiple limbs

## Reading and Understanding Humanoid URDF Files

Let's examine a simplified URDF for a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="tutorial_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.075 0.1 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin rpy="1.57 0 0"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 -0.15 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin rpy="1.57 0 0"/>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Additional joints and links for legs, right arm, etc. would follow -->
</robot>
```

### Key Components in Humanoid URDFs

1. **Links**: Represent rigid bodies (torso, head, limbs)
2. **Joints**: Define how links connect (revolute for rotational, prismatic for linear, etc.)
3. **Visual**: Defines how the robot appears in simulation
4. **Collision**: Defines collision geometry for physics simulation
5. **Inertial**: Physical properties for dynamics simulation

## Modifying Humanoid URDF Configurations

URDF files can be modified to change robot configurations:

### Adding Sensors

```xml
<!-- Add an IMU to the torso -->
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### Changing Physical Properties

```xml
<!-- Modify link properties -->
<link name="left_upper_arm">
  <visual>
    <geometry>
      <cylinder length="0.35" radius="0.05"/> <!-- Longer arm -->
    </geometry>
    <origin rpy="1.57 0 0"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.35" radius="0.05"/> <!-- Match visual -->
    </geometry>
    <origin rpy="1.57 0 0"/>
  </collision>
  <inertial>
    <mass value="1.7"/> <!-- Heavier arm -->
    <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.015"/>
  </inertial>
</link>
```

## ROS Controllers for Humanoid Robots

ROS controllers manage the low-level control of robot joints. For humanoid robots, we typically use:

### Joint State Controller
Publishes the current state of all joints.

### Position, Velocity, and Effort Controllers
Control joints using different control strategies.

### Example Controller Configuration

```yaml
# controller_manager.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: position_controllers/JointGroupPositionController

    right_leg_controller:
      type: position_controllers/JointGroupPositionController

    left_arm_controller:
      type: position_controllers/JointGroupPositionController

    right_arm_controller:
      type: position_controllers/JointGroupPositionController

# Individual controller configurations
left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint

right_leg_controller:
  ros__parameters:
    joints:
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint
```

## Python Scripts for Humanoid Control

### Basic Joint Control Script

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Timer for control loop
        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        # Initialize joint names for a humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        # Initialize joint positions
        self.joint_positions = [0.0] * len(self.joint_names)
        self.time_step = 0

    def control_loop(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Update joint positions based on some control logic
        self.update_joint_positions()
        msg.position = self.joint_positions

        # Publish joint commands
        self.joint_pub.publish(msg)

        self.get_logger().info(f'Published joint commands: {self.joint_positions[:3]}...')

    def update_joint_positions(self):
        # Example: Simple oscillating pattern for walking motion
        self.time_step += 0.1

        # Left leg pattern
        self.joint_positions[0] = 0.2 * math.sin(self.time_step)  # hip
        self.joint_positions[1] = 0.3 * math.sin(self.time_step * 2)  # knee
        self.joint_positions[2] = -0.1 * math.sin(self.time_step)  # ankle

        # Right leg pattern (opposite phase)
        self.joint_positions[3] = 0.2 * math.sin(self.time_step + math.pi)  # hip
        self.joint_positions[4] = 0.3 * math.sin(self.time_step * 2 + math.pi)  # knee
        self.joint_positions[5] = -0.1 * math.sin(self.time_step + math.pi)  # ankle

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Joint State Subscriber

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Note: actual state, not commands
            self.joint_state_callback,
            10)

        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        # Log joint positions
        position_dict = dict(zip(msg.name, msg.position))
        self.get_logger().info(f'Joint positions: {position_dict}')

def main(args=None):
    rclpy.init(args=args)
    monitor = JointStateMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## MoveIt! Integration for Motion Planning

MoveIt! is a powerful motion planning framework for ROS that's particularly useful for humanoid robots with many degrees of freedom.

### Basic MoveIt! Setup for Humanoid Arms

```python
import rclpy
from rclpy.node import Node
import moveit_commander
import geometry_msgs.msg
from math import pi

class HumanoidMotionPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_motion_planner')

        # Initialize MoveIt! commander
        moveit_commander.roscpp_initialize([])

        # Create robot commander
        self.robot = moveit_commander.RobotCommander()

        # Create scene interface
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create arm group commander
        self.arm_group = moveit_commander.MoveGroupCommander("left_arm")

    def move_arm_to_pose(self, x, y, z, roll, pitch, yaw):
        # Set target pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        # Plan and execute motion
        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.go(wait=True)

        # Stop and clear targets
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        return plan

def main(args=None):
    rclpy.init(args=args)

    try:
        planner = HumanoidMotionPlanner()

        # Move left arm to a specific pose
        success = planner.move_arm_to_pose(0.5, 0.2, 1.0, 0, pi/2, 0)

        if success:
            print("Motion planning and execution successful!")
        else:
            print("Motion planning failed!")

    except Exception as e:
        print(f"Error during motion planning: {e}")
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Integration for Humanoid Robots

Humanoid robots typically have various sensors including IMUs, force/torque sensors, cameras, and joint encoders.

### IMU Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        # Publisher for processed data
        self.balance_pub = self.create_publisher(Vector3, '/balance_status', 10)

    def imu_callback(self, msg):
        # Extract orientation from quaternion
        orientation = msg.orientation
        # Convert to roll, pitch, yaw for balance analysis

        # Simple balance check based on pitch
        pitch = self.quaternion_to_pitch(orientation)

        # Create balance status message
        balance_msg = Vector3()
        balance_msg.x = pitch  # Pitch angle
        balance_msg.y = msg.angular_velocity.y  # Angular velocity around y-axis
        balance_msg.z = msg.linear_acceleration.x  # Linear acceleration

        # Publish balance status
        self.balance_pub.publish(balance_msg)

        self.get_logger().info(f'Balance status - Pitch: {pitch:.2f}')

    def quaternion_to_pitch(self, q):
        # Convert quaternion to pitch angle
        sinr_cosp = 2 * (q.w * q.y - q.z * q.x)
        cosr_cosp = 1 - 2 * (q.y * q.y + q.x * q.x)
        pitch = math.atan2(sinr_cosp, cosr_cosp)
        return pitch

def main(args=None):
    rclpy.init(args=args)
    processor = ImuProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## AI Agent Integration with ROS Controllers

AI agents can interface with ROS controllers to make high-level decisions for humanoid robots.

### Basic AI Agent Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import random

class HumanoidAIAgent(Node):
    def __init__(self):
        super().__init__('humanoid_ai_agent')

        # Publishers for commands
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.behavior_pub = self.create_publisher(String, '/current_behavior', 10)

        # Subscribers for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Timer for AI decision making
        timer_period = 1.0  # 1Hz decision making
        self.timer = self.create_timer(timer_period, self.ai_decision_loop)

        # Internal state
        self.current_behavior = "idle"
        self.joint_states = {}
        self.imu_data = None

    def joint_state_callback(self, msg):
        self.joint_states = dict(zip(msg.name, msg.position))

    def imu_callback(self, msg):
        self.imu_data = msg

    def ai_decision_loop(self):
        # Simple AI decision based on sensor data
        if self.imu_data is not None:
            # Check if robot is balanced based on IMU data
            pitch = self.quaternion_to_pitch(self.imu_data.orientation)

            if abs(pitch) > 0.5:  # Robot is tilting too much
                self.current_behavior = "balance"
                self.execute_balance_behavior()
            elif random.random() < 0.1:  # 10% chance to change behavior
                self.current_behavior = random.choice(["walk", "wave", "idle"])
                self.execute_behavior(self.current_behavior)

        # Publish current behavior
        behavior_msg = String()
        behavior_msg.data = self.current_behavior
        self.behavior_pub.publish(behavior_msg)

        self.get_logger().info(f'AI Decision: {self.current_behavior}')

    def execute_balance_behavior(self):
        # Send joint commands to balance the robot
        cmd_msg = JointState()
        cmd_msg.name = list(self.joint_states.keys()) if self.joint_states else []

        # Simple balance correction (in reality, this would be more sophisticated)
        positions = []
        for joint_name in cmd_msg.name:
            if 'hip' in joint_name:
                positions.append(0.1 if 'left' in joint_name else -0.1)  # Adjust hip angles
            elif 'ankle' in joint_name:
                positions.append(-0.1 if 'left' in joint_name else 0.1)  # Adjust ankle angles
            else:
                positions.append(0.0)  # Default position

        cmd_msg.position = positions
        self.joint_cmd_pub.publish(cmd_msg)

    def execute_behavior(self, behavior):
        # Execute a specific behavior based on AI decision
        cmd_msg = JointState()

        if behavior == "walk":
            # Implement walking pattern
            pass
        elif behavior == "wave":
            # Implement waving pattern
            pass
        elif behavior == "idle":
            # Return to neutral position
            pass

def main(args=None):
    rclpy.init(args=args)
    agent = HumanoidAIAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Example: Simple Humanoid Walking Gait

Let's implement a complete example that demonstrates a simple walking gait for a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class SimpleWalkingGait(Node):
    def __init__(self):
        super().__init__('simple_walking_gait')

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Timer for control loop (50Hz)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.control_loop)

        # Joint names for a simple humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        # Walking gait parameters
        self.phase = 0.0
        self.step_frequency = 0.5  # Hz
        self.step_amplitude = 0.3  # Radians

        self.get_logger().info('Simple Walking Gait Node Initialized')

    def control_loop(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Calculate walking gait positions
        positions = self.calculate_walking_gait()
        msg.position = positions

        # Publish joint commands
        self.joint_pub.publish(msg)

        # Update phase for next iteration
        self.phase += 2 * math.pi * self.step_frequency * 0.02

        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi

    def calculate_walking_gait(self):
        # Calculate joint positions for a simple walking gait
        positions = [0.0] * len(self.joint_names)

        # Left leg: oscillate with phase
        positions[0] = self.step_amplitude * math.sin(self.phase)  # hip
        positions[1] = self.step_amplitude * 0.8 * math.sin(self.phase * 2)  # knee
        positions[2] = -self.step_amplitude * 0.5 * math.sin(self.phase)  # ankle

        # Right leg: oscillate with opposite phase
        positions[3] = self.step_amplitude * math.sin(self.phase + math.pi)  # hip
        positions[4] = self.step_amplitude * 0.8 * math.sin(self.phase * 2 + math.pi)  # knee
        positions[5] = -self.step_amplitude * 0.5 * math.sin(self.phase + math.pi)  # ankle

        return positions

def main(args=None):
    rclpy.init(args=args)
    walker = SimpleWalkingGait()

    try:
        rclpy.spin(walker)
    except KeyboardInterrupt:
        pass
    finally:
        walker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Controlling humanoid robots with ROS 2 involves understanding and integrating several key components:

1. **URDF**: The Unified Robot Description Format defines the physical structure of the robot
2. **Kinematics**: Understanding forward and inverse kinematics is crucial for controlling robot movements
3. **Controllers**: ROS controllers manage the low-level joint control
4. **Sensors**: IMUs, joint encoders, and other sensors provide feedback for control
5. **Motion Planning**: Frameworks like MoveIt! enable complex motion planning
6. **AI Integration**: High-level AI agents can make decisions based on sensor data

The combination of Python scripting with ROS 2's communication patterns allows for sophisticated control of humanoid robots. The modular architecture of ROS 2 makes it possible to develop, test, and integrate different components of a humanoid robot system independently.

In practice, controlling humanoid robots requires careful attention to stability, safety, and real-time performance. The examples in this chapter provide a foundation for understanding these concepts, but real-world applications will require more sophisticated control algorithms and safety measures.