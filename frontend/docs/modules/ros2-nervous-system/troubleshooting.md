---
sidebar_position: 5
---

# Troubleshooting Common ROS 2 Issues

This section provides solutions to common problems you may encounter when working with ROS 2, particularly in the context of humanoid robotics.

## General ROS 2 Issues

### Nodes Cannot Communicate Across Network

**Problem**: Nodes running on different machines cannot discover each other.

**Solutions**:
1. Ensure all machines are on the same network
2. Check firewall settings to allow DDS communication (typically UDP ports 7400-7500)
3. Set the `ROS_DOMAIN_ID` environment variable to the same value on all machines:
   ```bash
   export ROS_DOMAIN_ID=42
   ```
4. Verify network configuration with `ros2 doctor`:
   ```bash
   ros2 doctor --report
   ```

### Service Calls Timeout

**Problem**: Service clients fail with timeout errors.

**Solutions**:
1. Verify the service server is running:
   ```bash
   ros2 service list
   ```
2. Check if the service server is properly advertising the service:
   ```bash
   ros2 service info <service_name>
   ```
3. Increase the timeout in your service client:
   ```python
   while not self.cli.wait_for_service(timeout_sec=5.0):  # Increased from 1.0
       self.get_logger().info('Service not available, waiting again...')
   ```

### Topic Messages Not Being Received

**Problem**: Subscribers are not receiving messages published to topics.

**Solutions**:
1. Verify topic names match exactly (including case):
   ```bash
   ros2 topic list
   ros2 topic info <topic_name>
   ```
2. Check QoS profile compatibility between publisher and subscriber
3. Ensure publisher is actually sending messages by monitoring the topic:
   ```bash
   ros2 topic echo <topic_name>
   ```

## Humanoid-Specific Issues

### Joint Commands Not Executing

**Problem**: Joint command messages are published but joints don't move.

**Solutions**:
1. Verify the controller is running and properly configured:
   ```bash
   ros2 control list_controllers
   ```
2. Check that joint names in your command match those expected by the controller
3. Ensure the controller is in the correct state (active):
   ```bash
   ros2 control list_controllers | grep active
   ```

### Balance Issues in Simulation

**Problem**: Humanoid robot falls over in simulation.

**Solutions**:
1. Check that IMU sensors are properly configured in the URDF
2. Verify that center of mass is properly positioned
3. Ensure control frequency is high enough (typically 100Hz or higher for balance)
4. Check that PD gains for joint controllers are properly tuned

### High Joint Position Errors

**Problem**: Large differences between commanded and actual joint positions.

**Solutions**:
1. Increase controller effort limits in the controller configuration
2. Check if joint limits in URDF are too restrictive
3. Verify that the robot model in simulation matches the controller expectations
4. Adjust PD controller gains if using position controllers

## Performance Issues

### High CPU Usage

**Problem**: ROS 2 nodes consuming excessive CPU resources.

**Solutions**:
1. Reduce timer callback frequencies where possible
2. Use appropriate QoS settings (e.g., reduce history depth for non-critical topics)
3. Consider using multi-threaded executors for nodes with multiple callbacks:
   ```python
   executor = MultiThreadedExecutor()
   rclpy.spin(node, executor)
   ```

### Message Delays

**Problem**: Significant delays in message delivery.

**Solutions**:
1. Check network bandwidth if using distributed systems
2. Adjust QoS reliability settings if real-time performance is more important than reliability
3. Reduce message frequency or size if possible

## Development Environment Issues

### Package Not Found

**Problem**: `ros2 run` or `ros2 launch` cannot find your package.

**Solutions**:
1. Source the setup file after building:
   ```bash
   cd ~/ros2_workspace
   source install/setup.bash
   ```
2. Verify the package was built successfully:
   ```bash
   colcon build --packages-select <package_name>
   ```
3. Check that the package name matches exactly (case-sensitive)

### Import Errors in Python Nodes

**Problem**: Python nodes fail with import errors for ROS 2 modules.

**Solutions**:
1. Ensure ROS 2 Python packages are installed:
   ```bash
   pip3 install ros-foxy-ros2cli  # Replace 'foxy' with your ROS 2 distro
   ```
2. Check Python path is set correctly:
   ```bash
   echo $PYTHONPATH
   ```
3. Verify you're running Python 3.8 or higher

## Build and Installation Issues

### Colcon Build Fails

**Problem**: `colcon build` fails with compilation errors.

**Solutions**:
1. Ensure all dependencies are installed:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
2. Clean the build directory and rebuild:
   ```bash
   rm -rf build/ install/ log/
   colcon build
   ```
3. Check for missing package dependencies in `package.xml`

### Missing Message/Service Definitions

**Problem**: Custom message or service types are not found.

**Solutions**:
1. Ensure `.msg` and `.srv` files are in the correct directory (`msg/` or `srv/` in your package)
2. Add message generation dependencies to `package.xml`:
   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```
3. Add message generation to `CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/MyMessage.msg"
     "srv/MyService.srv"
   )
   ```

## Debugging Tips

### Enable Detailed Logging

To get more information about what's happening in your nodes:

```python
import rclpy
from rclpy.logging import LoggingSeverity

def main(args=None):
    rclpy.init(args=args)

    # Set logging level to DEBUG
    rclpy.logging.set_logger_level('your_node_name', LoggingSeverity.DEBUG)

    # Your node code here
```

### Monitor System Performance

Use these commands to monitor ROS 2 system performance:

```bash
# View all active nodes
ros2 node list

# View all topics and their types
ros2 topic list -t

# Monitor topic frequency
ros2 topic hz <topic_name>

# Monitor service calls
ros2 service list
```

### Common Debugging Workflow

1. Start with `ros2 doctor` to check system configuration
2. Use `ros2 node list` and `ros2 topic list` to verify system state
3. Monitor topics with `ros2 topic echo` to see actual data
4. Use `rqt_graph` to visualize the node graph:
   ```bash
   rqt_graph
   ```

If you encounter an issue not listed here, check the official ROS 2 documentation at https://docs.ros.org/ or the ROS Discourse forum for additional help.