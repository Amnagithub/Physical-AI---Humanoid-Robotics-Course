---
sidebar_position: 2
title: "Gazebo Setup and Installation"
---

# Gazebo Setup and Installation

This guide will walk you through setting up Gazebo for humanoid robot simulation. Gazebo is a powerful physics simulator widely used in robotics research and development.

## Prerequisites

Before installing Gazebo, ensure your system meets the following requirements:
- Ubuntu 20.04 LTS or later (or use a cloud-based solution)
- At least 4GB of RAM
- Graphics card with OpenGL 2.1 support
- At least 5GB of free disk space

## Installation Options

### Option 1: Native Installation (Ubuntu)

1. Update your package list:
   ```bash
   sudo apt update
   ```

2. Install Gazebo from packages:
   ```bash
   sudo apt install gazebo libgazebo-dev
   ```

3. For the latest version, you can add the OSRF repository:
   ```bash
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt update
   sudo apt install gazebo libgazebo-dev
   ```

### Option 2: Docker Installation

If you prefer using Docker to avoid system dependencies:

1. Pull the Gazebo Docker image:
   ```bash
   docker pull gazebo:gz-latest
   ```

2. Run Gazebo with GUI support:
   ```bash
   xhost +local:docker
   docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw gazebo:gz-latest gazebo
   ```

### Option 3: Cloud-Based Solutions

For those without suitable hardware, consider cloud-based solutions:
- AWS RoboMaker (with Gazebo integration)
- Google Cloud with virtual desktop
- GitHub Codespaces with X11 forwarding

## Verification

After installation, verify Gazebo is working by running:
```bash
gazebo
```

This should open the Gazebo simulator interface. If you encounter issues, check the troubleshooting section below.

## ROS Integration (Optional)

For full robotics functionality, you may want to install ROS (Robot Operating System) alongside Gazebo:

1. Install ROS Noetic (for Ubuntu 20.04) or ROS 2:
   ```bash
   sudo apt install ros-noetic-desktop
   ```

2. Install Gazebo ROS packages:
   ```bash
   sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
   ```

## Troubleshooting

### Common Issues

**Issue**: Gazebo fails to start with graphics errors
**Solution**: Ensure your graphics drivers are up to date. For virtual machines, enable 3D acceleration.

**Issue**: Low performance or lag
**Solution**: Reduce the rendering quality in Gazebo settings or use a more powerful machine.

**Issue**: Missing plugins
**Solution**: Install additional Gazebo plugins:
   ```bash
   sudo apt install gazebo9-plugins libgazebo9-dev
   ```

## Next Steps

Once Gazebo is properly installed and configured, continue to the next section to learn about physics concepts in simulation.