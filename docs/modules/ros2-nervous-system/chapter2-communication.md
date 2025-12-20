---
sidebar_position: 2
---

# Chapter 2: Core ROS 2 Communication Concepts (Nodes, Topics, Services)

## Learning Objectives

After completing this chapter, you should be able to:
- Create and run ROS 2 nodes using Python (rclpy)
- Implement publisher-subscriber communication using topics
- Create and use services for request-response communication
- Understand the lifecycle of ROS 2 nodes
- Apply parameter servers for configuration management
- Use actions for complex, long-running tasks
- Implement custom message types

## Table of Contents
- [Introduction to ROS 2 Communication](#introduction-to-ros-2-communication)
- [ROS 2 Nodes and Lifecycle](#ros-2-nodes-and-lifecycle)
- [Topics and Publisher-Subscriber Pattern](#topics-and-publisher-subscriber-pattern)
- [Services and Request-Response Pattern](#services-and-request-response-pattern)
- [Parameter Servers](#parameter-servers)
- [Actions for Complex Tasks](#actions-for-complex-tasks)
- [Message Types and Custom Messages](#message-types-and-custom-messages)
- [Practical Examples with Humanoid Robots](#practical-examples-with-humanoid-robots)
- [Summary](#summary)

## Introduction to ROS 2 Communication

Communication is the nervous system's primary function, and in ROS 2, communication between different parts of a robotic system is facilitated through several patterns. The three primary communication patterns are:

1. **Topics** - for asynchronous, one-way data streaming (publish-subscribe)
2. **Services** - for synchronous request-response interactions
3. **Actions** - for long-running tasks with feedback and goal management

These patterns allow for loose coupling between different components of a robotic system, enabling modular and maintainable robot software.

## ROS 2 Nodes and Lifecycle

A node is the fundamental building block of a ROS 2 system. It's a process that performs computation and participates in the ROS 2 communication system.

### Node Creation in Python

```python
import rclpy
from rclpy.node import Node

class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle States

ROS 2 nodes have a lifecycle with several states:
- **Unconfigured**: Initial state
- **Inactive**: Configured but not executing
- **Active**: Running and participating in communication
- **Finalized**: Being shut down

This lifecycle allows for better resource management and system stability.

## Topics and Publisher-Subscriber Pattern

Topics enable asynchronous communication through a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics.

### Creating a Publisher Node

For humanoid robots, we often need to publish more complex data structures like joint states. Here's a more realistic example for humanoid robot control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class HumanoidJointCommandPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_joint_command_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_commands', 10)
        timer_period = 0.02  # 50Hz for real-time control
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Define joint names for a humanoid robot
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.time_step = 0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Generate coordinated joint positions for a walking motion
        positions = []
        for i, joint_name in enumerate(self.joint_names):
            # Different oscillation patterns for different joints
            if 'hip' in joint_name:
                pos = 0.2 * math.sin(self.time_step * 2.0 + i * 0.5)
            elif 'knee' in joint_name:
                pos = 0.3 * math.sin(self.time_step * 4.0 + i * 0.3)
            elif 'ankle' in joint_name:
                pos = 0.1 * math.sin(self.time_step * 2.0 + i * 0.7)
            elif 'shoulder' in joint_name:
                pos = 0.1 * math.sin(self.time_step * 1.0 + i * 0.2)
            elif 'elbow' in joint_name:
                pos = 0.15 * math.sin(self.time_step * 1.5 + i * 0.4)
            else:
                pos = 0.0

            positions.append(pos)

        msg.position = positions
        msg.velocity = [0.0] * len(positions)  # Initially zero velocity
        msg.effort = [0.0] * len(positions)    # Initially zero effort

        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint commands for {len(self.joint_names)} joints')
        self.time_step += 0.02

def main(args=None):
    rclpy.init(args=args)
    publisher = HumanoidJointCommandPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber Node

For humanoid robots, we typically subscribe to joint states to monitor the current position, velocity, and effort of each joint:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class HumanoidStateSubscriber(Node):
    def __init__(self):
        super().__init__('humanoid_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Topic for actual joint states from robot
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Store the most recent joint state
        self.current_joint_state = None
        self.last_update_time = time.time()

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.last_update_time = time.time()

        # Log the current joint positions
        position_dict = dict(zip(msg.name, msg.position))
        self.get_logger().info(f'Humanoid joint positions: {position_dict}')

        # Check for any joint that might be out of normal range (safety check)
        for name, pos in zip(msg.name, msg.position):
            if abs(pos) > 3.0:  # Unusually high joint angle
                self.get_logger().warn(f'Joint {name} has unusual position: {pos}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = HumanoidStateSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS) Settings

QoS settings allow you to fine-tune the communication characteristics:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For real-time humanoid control
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

## Services and Request-Response Pattern

Services provide synchronous request-response communication. A service client sends a request and waits for a response from a service server. This is useful for humanoid robots when you need to execute specific behaviors with guaranteed completion.

### Creating a Service Server

Here's an example of a service server for executing humanoid behaviors:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class HumanoidBehaviorServer(Node):
    def __init__(self):
        super().__init__('humanoid_behavior_server')
        self.srv = self.create_service(
            Trigger,
            'execute_behavior',
            self.execute_behavior_callback
        )

        # Define available behaviors
        self.behaviors = {
            'wave': self.execute_wave_behavior,
            'step_forward': self.execute_step_forward,
            'balance': self.execute_balance_behavior,
            'sit': self.execute_sit_behavior,
            'stand': self.execute_stand_behavior
        }

        self.get_logger().info('Humanoid Behavior Server initialized')

    def execute_behavior_callback(self, request, response):
        behavior_name = request.message if request.message else 'wave'

        if behavior_name in self.behaviors:
            self.get_logger().info(f'Executing behavior: {behavior_name}')

            # Execute the requested behavior
            success = self.behaviors[behavior_name]()

            response.success = success
            response.message = f'Behavior {behavior_name} completed' if success else f'Behavior {behavior_name} failed'
        else:
            response.success = False
            response.message = f'Unknown behavior: {behavior_name}. Available: {list(self.behaviors.keys())}'

        return response

    def execute_wave_behavior(self):
        # Simulate waving behavior - in real robot, this would send joint commands
        self.get_logger().info('Executing wave behavior')
        # In a real implementation, this would coordinate multiple joints
        time.sleep(2)  # Simulate time for behavior
        return True

    def execute_step_forward(self):
        self.get_logger().info('Executing step forward behavior')
        # Coordinate legs for stepping motion
        time.sleep(3)  # Simulate time for behavior
        return True

    def execute_balance_behavior(self):
        self.get_logger().info('Executing balance behavior')
        # Adjust joint positions to maintain balance
        time.sleep(1.5)  # Simulate time for behavior
        return True

    def execute_sit_behavior(self):
        self.get_logger().info('Executing sit behavior')
        # Coordinate joints for sitting motion
        time.sleep(4)  # Simulate time for behavior
        return True

    def execute_stand_behavior(self):
        self.get_logger().info('Executing stand behavior')
        # Coordinate joints for standing motion
        time.sleep(4)  # Simulate time for behavior
        return True

def main(args=None):
    rclpy.init(args=args)
    service_server = HumanoidBehaviorServer()

    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        pass
    finally:
        service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

Here's how to create a service client to request humanoid behaviors:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys

class HumanoidBehaviorClient(Node):
    def __init__(self):
        super().__init__('humanoid_behavior_client')
        self.cli = self.create_client(Trigger, 'execute_behavior')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Behavior service not available, waiting again...')

        self.req = Trigger.Request()

    def execute_behavior(self, behavior_name):
        self.req.message = behavior_name
        self.future = self.cli.call_async(self.req)

        # Wait for the response
        rclpy.spin_until_future_complete(self, self.future)

        try:
            response = self.future.result()
            if response.success:
                self.get_logger().info(f'Successfully executed behavior: {response.message}')
            else:
                self.get_logger().error(f'Failed to execute behavior: {response.message}')

            return response.success
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    client = HumanoidBehaviorClient()

    # Get behavior name from command line argument or default to 'wave'
    behavior_name = sys.argv[1] if len(sys.argv) > 1 else 'wave'

    try:
        success = client.execute_behavior(behavior_name)
        if success:
            print(f"Behavior '{behavior_name}' executed successfully!")
        else:
            print(f"Failed to execute behavior '{behavior_name}'")
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameter Servers

Parameters provide a way to configure nodes at runtime. They're useful for tuning values without recompiling code.

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('max_joint_velocity', 1.0)

        # Get parameter values
        self.frequency = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_joint_velocity').value

        self.get_logger().info(f'Frequency: {self.frequency}, Max velocity: {self.max_velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions for Complex Tasks

Actions are used for long-running tasks that require feedback and goal management. They're particularly useful for humanoid robot behaviors that take time to complete, like walking across a room or performing a complex manipulation task.

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

# For this example, we'll create a custom action message structure
# In practice, you would define this in an .action file and generate the message
class WalkGoal:
    def __init__(self):
        self.steps = 10  # Number of steps to take
        self.speed = 1.0  # Speed multiplier

class WalkFeedback:
    def __init__(self):
        self.current_step = 0
        self.distance_traveled = 0.0
        self.balance_status = "stable"

class WalkResult:
    def __init__(self):
        self.success = True
        self.total_steps = 0
        self.distance_moved = 0.0

class HumanoidWalkActionServer(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_server')

        # Using a reentrant callback group to handle multiple callbacks
        callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            # In a real implementation, you would use a generated action type
            # For this example, we're simulating the structure
            self._execute_callback,
            'walk_action',
            callback_group=callback_group,
            cancel_callback=self._cancel_callback
        )

        self.get_logger().info('Humanoid Walk Action Server initialized')

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for walk action')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing walk action for {goal_handle.request.steps} steps')

        # Initialize feedback
        feedback_msg = WalkFeedback()
        result = WalkResult()

        # Simulate walking process
        for step in range(goal_handle.request.steps):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.total_steps = step
                self.get_logger().info('Walk action canceled')
                return result

            # Simulate walking step
            time.sleep(0.5 / goal_handle.request.speed)  # Adjust for speed

            # Update feedback
            feedback_msg.current_step = step + 1
            feedback_msg.distance_traveled = step * 0.3  # Assume 0.3m per step
            feedback_msg.balance_status = "stable" if step % 3 != 0 else "adjusting"  # Simulate balance adjustments

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Step {step + 1}/{goal_handle.request.steps}, '
                                 f'Distance: {feedback_msg.distance_traveled:.2f}m')

        # Complete the action
        goal_handle.succeed()
        result.success = True
        result.total_steps = goal_handle.request.steps
        result.distance_moved = goal_handle.request.steps * 0.3
        self.get_logger().info(f'Walk action completed: moved {result.distance_moved:.2f}m')

        return result

def main(args=None):
    rclpy.init(args=args)

    # Use a multi-threaded executor to handle action callbacks
    node = HumanoidWalkActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

For completeness, here's how a client would interact with the action server:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

class HumanoidWalkActionClient(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_client')

        # In a real implementation, you would use a generated action type
        self._action_client = ActionClient(
            self,
            # action_type,  # Would be the generated action type
            'walk_action'
        )

    def send_goal(self, steps=5, speed=1.0):
        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create the goal
        goal_msg = WalkGoal()  # In reality, this would be the generated goal message
        goal_msg.steps = steps
        goal_msg.speed = speed

        # Send the goal and get a future
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        # Wait for the result
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result...')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        self.get_logger().info(f'Result: {result.success}, Total steps: {result.total_steps}')

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Current step: {feedback.current_step}, '
            f'Distance: {feedback.distance_traveled:.2f}m, '
            f'Balance: {feedback.balance_status}'
        )

def main(args=None):
    rclpy.init(args=args)
    client = HumanoidWalkActionClient()

    try:
        # Send a goal to walk 10 steps at normal speed
        client.send_goal(steps=10, speed=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Message Types and Custom Messages

ROS 2 provides standard message types but also allows you to define custom message types for specific applications.

### Common Standard Messages
- `std_msgs`: Basic data types (Int32, String, Float64, etc.)
- `sensor_msgs`: Sensor data (JointState, Image, LaserScan, etc.)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- `nav_msgs`: Navigation-specific messages

### Creating Custom Messages

Custom messages are defined in `.msg` files in the `msg/` directory of a package:

```
# JointCommand.msg
string joint_name
float64 position
float64 velocity
float64 effort
```

## Hands-on Exercise: Creating a Simple Communication System (T040)

Let's create a complete example that demonstrates how to integrate multiple communication patterns for humanoid robot control. This exercise will combine topics, services, and parameters to create a coordinated system.

### Exercise: Humanoid Robot Commander Node

This node will:
1. Subscribe to sensor data (topics)
2. Provide services to request specific actions
3. Use parameters to configure behavior
4. Publish commands to the robot

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult
import math

class HumanoidRobotCommander(Node):
    def __init__(self):
        super().__init__('humanoid_robot_commander')

        # Declare parameters with default values
        self.declare_parameter('control_frequency', 50)  # Hz
        self.declare_parameter('safety_threshold', 0.5)  # Radians
        self.declare_parameter('max_joint_velocity', 2.0)  # rad/s
        self.declare_parameter('balance_check_enabled', True)

        # Get parameter values
        self.control_freq = self.get_parameter('control_frequency').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.max_velocity = self.get_parameter('max_joint_velocity').value
        self.balance_check_enabled = self.get_parameter('balance_check_enabled').value

        # Create publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # Create subscribers for sensor data
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Create service servers
        self.stand_service = self.create_service(Trigger, 'stand_up', self.stand_callback)
        self.sit_service = self.create_service(Trigger, 'sit_down', self.sit_callback)
        self.balance_service = self.create_service(Trigger, 'check_balance', self.balance_callback)

        # Timer for control loop
        timer_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        # Internal state
        self.current_joint_states = JointState()
        self.imu_data = None
        self.robot_state = "idle"  # idle, standing, sitting, walking

        self.get_logger().info('Humanoid Robot Commander initialized')

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def imu_callback(self, msg):
        self.imu_data = msg

    def control_loop(self):
        # Main control loop that monitors robot state and safety
        if self.balance_check_enabled and self.imu_data is not None:
            # Check if robot is tilting too much
            pitch = self.get_pitch_from_imu(self.imu_data)
            if abs(pitch) > 0.5:  # Too much tilt
                self.get_logger().warn(f'Robot is tilting! Pitch: {pitch:.3f}')
                # Emergency safety stop - send zero commands
                self.send_zero_commands()

        # Log current state periodically
        if self.get_clock().now().nanoseconds % 5000000000 == 0:  # Every 5 seconds
            self.get_logger().info(f'Robot state: {self.robot_state}, Joint count: {len(self.current_joint_states.name)}')

    def get_pitch_from_imu(self, imu_msg):
        # Convert quaternion to pitch angle
        w, x, y, z = imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z
        sinr_cosp = 2 * (w * y - z * x)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        pitch = math.atan2(sinr_cosp, cosr_cosp)
        return pitch

    def stand_callback(self, request, response):
        self.get_logger().info('Received stand up command')

        if self.robot_state == "sitting":
            # Generate standing motion
            self.execute_standing_motion()
            response.success = True
            response.message = "Standing motion initiated"
            self.robot_state = "standing"
        else:
            response.success = False
            response.message = f"Cannot stand from {self.robot_state} state"

        return response

    def sit_callback(self, request, response):
        self.get_logger().info('Received sit down command')

        if self.robot_state == "standing":
            # Generate sitting motion
            self.execute_sitting_motion()
            response.success = True
            response.message = "Sitting motion initiated"
            self.robot_state = "sitting"
        else:
            response.success = False
            response.message = f"Cannot sit from {self.robot_state} state"

        return response

    def balance_callback(self, request, response):
        self.get_logger().info('Received balance check command')

        if self.imu_data is not None:
            pitch = self.get_pitch_from_imu(self.imu_data)
            if abs(pitch) < 0.1:
                response.success = True
                response.message = f"Robot is well balanced. Pitch: {pitch:.3f}"
            else:
                response.success = False
                response.message = f"Robot is off balance. Pitch: {pitch:.3f}"
        else:
            response.success = False
            response.message = "No IMU data available for balance check"

        return response

    def execute_standing_motion(self):
        # In a real implementation, this would send coordinated joint commands
        # to transition from sitting to standing position
        self.get_logger().info('Executing standing motion...')
        # Send joint commands to stand up position
        self.send_target_joint_positions(self.get_standing_positions())

    def execute_sitting_motion(self):
        # In a real implementation, this would send coordinated joint commands
        # to transition from standing to sitting position
        self.get_logger().info('Executing sitting motion...')
        # Send joint commands to sit down position
        self.send_target_joint_positions(self.get_sitting_positions())

    def get_standing_positions(self):
        # Return joint positions for standing posture
        positions = {}
        for name in self.current_joint_states.name:
            if 'hip' in name:
                positions[name] = 0.0
            elif 'knee' in name:
                positions[name] = 0.0
            elif 'ankle' in name:
                positions[name] = 0.0
            elif 'shoulder' in name:
                positions[name] = 0.2  # Arms slightly forward for balance
            elif 'elbow' in name:
                positions[name] = -0.5  # Elbows bent
            else:
                positions[name] = 0.0
        return positions

    def get_sitting_positions(self):
        # Return joint positions for sitting posture
        positions = {}
        for name in self.current_joint_states.name:
            if 'hip' in name:
                positions[name] = 1.0  # Hips bent for sitting
            elif 'knee' in name:
                positions[name] = 1.0  # Knees bent
            elif 'ankle' in name:
                positions[name] = -0.5  # Feet position for sitting
            else:
                positions[name] = 0.0
        return positions

    def send_target_joint_positions(self, target_positions_dict):
        # Send joint position commands to the robot
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        # Set names and positions based on current joint names
        cmd_msg.name = self.current_joint_states.name
        cmd_msg.position = []

        for joint_name in cmd_msg.name:
            if joint_name in target_positions_dict:
                cmd_msg.position.append(target_positions_dict[joint_name])
            else:
                cmd_msg.position.append(0.0)  # Default to 0 if not specified

        cmd_msg.velocity = [0.0] * len(cmd_msg.position)  # Zero velocity targets
        cmd_msg.effort = [0.0] * len(cmd_msg.position)    # Zero effort targets

        self.joint_cmd_publisher.publish(cmd_msg)

    def send_zero_commands(self):
        # Send zero commands to stop all joints (safety feature)
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = self.current_joint_states.name
        cmd_msg.position = [0.0] * len(cmd_msg.name)
        cmd_msg.velocity = [0.0] * len(cmd_msg.name)
        cmd_msg.effort = [0.0] * len(cmd_msg.name)

        self.joint_cmd_publisher.publish(cmd_msg)
        self.get_logger().warn('Safety stop: sent zero commands to all joints')

def main(args=None):
    rclpy.init(args=args)
    commander = HumanoidRobotCommander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        commander.get_logger().info('Shutting down Humanoid Robot Commander...')
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This exercise demonstrates how to create a complete communication system that integrates:
- Topic subscriptions for sensor data
- Service servers for behavior requests
- Parameter configuration for runtime adjustments
- Topic publishing for control commands

To run this system, you would need to have a simulated or real humanoid robot publishing joint states and IMU data on the appropriate topics.

## Practical Examples with Humanoid Robots

### Publisher Example: Joint Command Publisher

For humanoid robots, we often need to send commands to multiple joints simultaneously:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class HumanoidCommandPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_command_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # Timer for sending commands at 50Hz
        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize joint names for a typical humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)  # Set all positions to 0
        msg.velocity = [0.0] * len(self.joint_names)  # Set all velocities to 0
        msg.effort = [0.0] * len(self.joint_names)   # Set all efforts to 0

        # Add a simple oscillating pattern to demonstrate control
        import math
        for i in range(len(msg.position)):
            msg.position[i] = 0.5 * math.sin(self.i * 0.1 + i * 0.5)

        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint commands - Cycle: {self.i}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = HumanoidCommandPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Example: Behavior Execution

A service for executing predefined humanoid behaviors:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BehaviorExecutionService(Node):
    def __init__(self):
        super().__init__('behavior_execution_service')
        self.srv = self.create_service(
            Trigger,
            'execute_behavior',
            self.execute_behavior_callback)

        # Define some simple behaviors
        self.behaviors = {
            'wave': self.execute_wave_behavior,
            'step': self.execute_step_behavior,
            'balance': self.execute_balance_behavior
        }

    def execute_behavior_callback(self, request, response):
        # In a real implementation, this would execute the requested behavior
        behavior_name = request.message if request.message else 'wave'

        if behavior_name in self.behaviors:
            self.get_logger().info(f'Executing behavior: {behavior_name}')
            success = self.behaviors[behavior_name]()
            response.success = success
            response.message = f'Behavior {behavior_name} completed' if success else f'Behavior {behavior_name} failed'
        else:
            response.success = False
            response.message = f'Unknown behavior: {behavior_name}'

        return response

    def execute_wave_behavior(self):
        # Implementation for waving behavior
        self.get_logger().info('Executing wave behavior')
        return True

    def execute_step_behavior(self):
        # Implementation for stepping behavior
        self.get_logger().info('Executing step behavior')
        return True

    def execute_balance_behavior(self):
        # Implementation for balancing behavior
        self.get_logger().info('Executing balance behavior')
        return True

def main(args=None):
    rclpy.init(args=args)
    service = BehaviorExecutionService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 communication patterns provide the foundation for creating complex robotic systems. The publish-subscribe pattern is ideal for streaming sensor data and control commands, while services provide synchronous request-response interactions for discrete actions. Actions are perfect for long-running tasks that require feedback and goal management.

For humanoid robots, these communication patterns enable the coordination of multiple joints, sensors, and control systems. The ability to create custom message types allows for precise control and monitoring of humanoid-specific components.

In the next chapter, we'll explore how to control humanoid robots using Python and URDF (Unified Robot Description Format), building on the communication concepts learned here.