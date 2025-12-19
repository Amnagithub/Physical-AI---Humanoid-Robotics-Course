---
sidebar_position: 4
---

# Glossary of ROS 2 and Robotics Terms

This glossary provides definitions for key terms used throughout the "ROS 2: The Robotic Nervous System" module.

## A

**Action**: A communication pattern in ROS 2 for long-running tasks that require feedback and goal management. Actions are ideal for behaviors that take time to complete, such as walking or manipulation tasks.

**API (Application Programming Interface)**: A set of rules and protocols for building and interacting with software applications. In ROS 2, APIs allow nodes to communicate with each other.

## B

**Behavior**: A specific action or set of coordinated movements that a robot can perform, such as walking, waving, or sitting.

## C

**Controller**: A ROS 2 component that manages the low-level control of robot joints, typically implementing position, velocity, or effort control.

**Coordinate Frame**: A system for specifying positions and orientations in 3D space, essential for robotics applications.

## D

**DDS (Data Distribution Service)**: A specification that defines a standard middleware for real-time, scalable, dependable data exchanges. DDS serves as the underlying communication layer in ROS 2.

**Dependency Injection**: A design pattern used in ROS 2 where a class receives its dependencies from an external source rather than creating them itself.

## E

**Ecosystem**: The collection of tools, packages, libraries, and community resources that support ROS 2 development.

**Executor**: In ROS 2, an executor controls which callbacks are called when. It manages the execution of callbacks from subscriptions, services, timers, etc.

## G

**Gazebo**: A physics-based simulation environment commonly used with ROS 2 for testing and validating robotic systems.

## I

**IMU (Inertial Measurement Unit)**: A sensor that measures specific force, angular rate, and sometimes the magnetic field surrounding the robot, used for balance and orientation.

**Interface**: In ROS 2, the definition of a service, action, or message type that allows nodes to communicate with each other.

## J

**Joint**: A connection between two links in a robot that allows relative motion between them.

**Joint State**: Information about the position, velocity, and effort of a robot's joints, typically published on the `/joint_states` topic.

## L

**Lifecycle**: The state machine that ROS 2 nodes can implement to manage their initialization, activation, and deactivation in a controlled manner.

## M

**Middleware**: Software that provides common services and capabilities to applications beyond what's offered by the operating system. In the context of ROS 2, it handles communication between distributed processes.

**MoveIt!**: A motion planning framework for ROS that provides capabilities for arm and full-body motion planning, inverse kinematics, collision checking, and visualization.

## N

**Node**: A process that performs computation in a ROS 2 system. Nodes are the fundamental building blocks of a ROS 2 application.

## P

**Package**: The software container in ROS 2 that contains libraries, executables, scripts, or other files required for a specific functionality.

**Parameter**: A configuration value that can be set at runtime for a ROS 2 node, allowing for dynamic reconfiguration without recompilation.

**Publisher**: A ROS 2 entity that sends messages to a topic in the publish-subscribe communication pattern.

## Q

**QoS (Quality of Service)**: Settings in ROS 2 that define how messages are delivered, including reliability, durability, deadline, and liveliness policies.

## R

**ROS (Robot Operating System)**: A flexible framework for writing robot software, providing a collection of tools, libraries, and conventions for simplifying robot development.

**ROS 2**: The second generation of the Robot Operating System, featuring improved real-time capabilities, multi-robot support, and better security.

**RViz**: A 3D visualization tool for displaying robot data in ROS, commonly used for debugging and monitoring robotic systems.

## S

**Service**: A synchronous request-response communication pattern in ROS 2, where a client sends a request and waits for a response from a server.

**Subscriber**: A ROS 2 entity that receives messages from a topic in the publish-subscribe communication pattern.

## T

**Topic**: A named bus over which nodes exchange messages in the publish-subscribe communication pattern.

**Transform**: A mathematical representation of the position and orientation of one coordinate frame relative to another, used in robot spatial reasoning.

## U

**URDF (Unified Robot Description Format)**: An XML-based format used in ROS to describe robot models, including links, joints, and other properties.

## V

**Velocity Controller**: A type of controller that commands joint velocities rather than positions.

## W

**Workspace**: A directory containing multiple ROS 2 packages that are built together using colcon.

## X, Y, Z

**Yaw**: Rotation around the vertical (Z) axis of a robot's coordinate frame.

**Pitch**: Rotation around the lateral (Y) axis of a robot's coordinate frame.

**Roll**: Rotation around the longitudinal (X) axis of a robot's coordinate frame.