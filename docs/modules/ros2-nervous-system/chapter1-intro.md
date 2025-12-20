---
sidebar_position: 1
---

# Chapter 1: Introduction to ROS 2 as a Robotic Middleware

## Learning Objectives

After completing this chapter, you should be able to:
- Explain what ROS 2 is and its role as middleware in robotic systems
- Describe the key components of the ROS 2 architecture
- Understand the importance of DDS (Data Distribution Service) in ROS 2
- Articulate why ROS 2 is critical for humanoid robotics applications

## Table of Contents
- [What is ROS 2?](#what-is-ros-2)
- [ROS 2 Architecture and Components](#ros-2-architecture-and-components)
- [DDS: The Communication Backbone](#dds-the-communication-backbone)
- [ROS 2 for Humanoid Robotics](#ros-2-for-humanoid-robotics)
- [ROS 2 Distributions](#ros-2-distributions)
- [ROS 2 Client Libraries](#ros-2-client-libraries)
- [ROS 2 Ecosystem](#ros-2-ecosystem)
- [Summary](#summary)

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike traditional software applications that run on a single computer, robots typically require multiple computers or processors to handle sensing, actuation, and computation. ROS 2 provides a middleware that allows these distributed processes to communicate with each other, regardless of the programming language they are written in or the operating system they run on.

### The Middleware Concept

A middleware is software that provides common services and capabilities to applications beyond what's offered by the operating system. In the context of robotics, middleware like ROS 2 provides:

- Message passing between processes
- Hardware abstraction
- Device drivers
- Libraries for implementing commonly used functionality
- Tools for visualization, simulation, and testing

## ROS 2 Architecture and Components

The ROS 2 architecture is built around several key components that work together to provide a flexible and robust framework for robot development:

### Nodes
Nodes are the fundamental building blocks of ROS 2 applications. Each node is a process that performs computation. Nodes written in different programming languages can be combined together in a single system.

### Packages
Packages are the software containers in ROS 2. They contain libraries, executables, scripts, or other files required for a specific functionality.

### Topics and Messages
Topics enable asynchronous message-passing between nodes. Messages are the data packets sent via topics between nodes.

### Services
Services provide a synchronous request/response communication pattern between nodes.

### Actions
Actions are a more complex communication pattern that allows for long-running tasks with feedback and goal management.

## DDS: The Communication Backbone

DDS (Data Distribution Service) is a specification that defines a standard middleware for real-time, scalable, dependable data exchanges. In ROS 2, DDS serves as the underlying communication layer that handles:

- Discovery: How nodes find each other on the network
- Data delivery: How messages are transmitted between nodes
- Quality of Service (QoS): How the system handles reliability, durability, and other communication properties

### DDS Implementation Options

ROS 2 supports multiple DDS implementations, including:
- Fast DDS (formerly Fast RTPS)
- Cyclone DDS
- RTI Connext DDS
- Eclipse iceoryx (for intra-process communication)

## ROS 2 for Humanoid Robotics

Humanoid robots present unique challenges that make ROS 2 particularly valuable:

### Complexity Management
Humanoid robots typically have dozens of joints, multiple sensors (cameras, IMUs, force/torque sensors), and complex control systems. ROS 2's distributed architecture allows these components to be managed by separate nodes that can run on different processors.

### Real-time Requirements
Many aspects of humanoid robot control require real-time performance. ROS 2's QoS settings allow for fine-tuning of communication characteristics to meet real-time requirements.

### Multi-robot Systems
Humanoid robots often need to coordinate with other robots or systems. ROS 2's networking capabilities facilitate this coordination.

## ROS 2 Distributions

ROS 2 follows a time-based release model with Long Term Support (LTS) distributions. For this course, we'll focus on ROS 2 Humble Hawksbill, which is an LTS distribution released in May 2022 with support until May 2027.

### Why Humble Hawksbill?
- Long-term support (5 years)
- Extensive documentation and community support
- Stability for educational purposes
- Active development and regular updates

## ROS 2 Client Libraries

ROS 2 provides client libraries that allow you to write ROS 2 code in different programming languages:

### rclcpp
The C++ client library, offering high performance for computationally intensive tasks.

### rclpy
The Python client library, which we'll focus on in this course due to its accessibility for learning and rapid prototyping.

### Other Languages
ROS 2 also supports Rust, Java, and other languages through community-maintained client libraries.

## ROS 2 Ecosystem

The ROS 2 ecosystem includes a wide range of tools and packages:

### Development Tools
- `ros2 run`: Run a node
- `ros2 topic`: Inspect and interact with topics
- `ros2 service`: Inspect and interact with services
- `rqt`: Graphical user interface for ROS 2

### Simulation
- Gazebo: Physics-based simulation environment
- RViz: 3D visualization tool for robot data

### Standard Packages
- Navigation: Path planning and obstacle avoidance
- Perception: Computer vision and sensor processing
- Control: Joint and trajectory controllers

## Hands-On Example: Basic ROS 2 Workspace Setup

Let's create a simple ROS 2 workspace and run our first example. This will help you understand the basic structure and workflow.

### Creating a Workspace

First, create a directory for your ROS 2 workspace:

```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
```

### Building the Workspace

To build the workspace, use the colcon build tool:

```bash
colcon build
source install/setup.bash
```

### Simple Publisher/Subscriber Example

Let's create a simple publisher and subscriber to understand the basics:

**Publisher Code (publisher_member_function.cpp):**

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
```

**Python Equivalent:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run the publisher:
```bash
ros2 run demo_nodes_cpp talker
# or for Python
ros2 run demo_nodes_py talker
```

To run the subscriber in another terminal:
```bash
ros2 run demo_nodes_cpp listener
# or for Python
ros2 run demo_nodes_py listener
```

## Simple ROS 2 Publisher/Subscriber Concept Demonstration (T018)

Let's look at a more humanoid-focused example. Here's a simple publisher that publishes joint commands and a subscriber that listens to joint states - a common pattern in humanoid robotics:

**Joint Command Publisher (Python):**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Define joint names for a simple humanoid leg
        self.joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']

    def timer_callback(self):
        msg = JointState()
        msg.name = self.joint_names

        # Create a simple oscillating pattern for walking simulation
        msg.position = [
            0.2 * math.sin(self.i * 0.2),      # hip
            0.3 * math.sin(self.i * 0.4),      # knee
            -0.1 * math.sin(self.i * 0.2)      # ankle
        ]

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint commands: {msg.position}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = JointCommandPublisher()

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

**Joint State Subscriber (Python):**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  # Different topic for actual joint states
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Process joint state information
        position_dict = dict(zip(msg.name, msg.position))
        self.get_logger().info(f'Joint positions: {position_dict}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = JointStateSubscriber()

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

This example demonstrates the publisher-subscriber pattern applied to humanoid robot joint control, which is fundamental to how ROS 2 manages communication in robotic systems.

## Exercises and Review Questions for Chapter 1 (T020)

1. What is the primary role of ROS 2 in a robotic system?
2. Explain the difference between ROS 1 and ROS 2 in terms of architecture.
3. What is DDS and why is it important for ROS 2?
4. List three advantages of using ROS 2 for humanoid robotics.
5. Describe the publish-subscribe communication pattern and give an example of where it might be used in a humanoid robot.

## Summary

ROS 2 serves as the "nervous system" of modern robotic systems, providing the communication infrastructure that allows complex robots to function as integrated systems. For humanoid robots, with their complex multi-joint systems and real-time control requirements, ROS 2 provides an ideal framework for managing complexity while enabling sophisticated behaviors.

In the next chapter, we'll dive deeper into the core communication concepts of ROS 2, including nodes, topics, and services.