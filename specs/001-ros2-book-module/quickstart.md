# Quickstart Guide: ROS 2 Book Module Development

## Prerequisites

Before working on the ROS 2 book module, ensure you have:

- Git installed
- Node.js (v16 or higher) and npm
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic understanding of ROS 2 concepts

## Setup Development Environment

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install Docusaurus dependencies:**
   ```bash
   npm install
   ```

3. **Verify ROS 2 installation:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 --version
   ```

## Create the Module Structure

1. **Create the docs directory if it doesn't exist:**
   ```bash
   mkdir -p docs/modules/ros2-nervous-system
   ```

2. **Create the three chapter files:**
   ```bash
   touch docs/modules/ros2-nervous-system/chapter1-intro.md
   touch docs/modules/ros2-nervous-system/chapter2-communication.md
   touch docs/modules/ros2-nervous-system/chapter3-humanoid-control.md
   ```

## Develop Content

1. **Start the Docusaurus development server:**
   ```bash
   npm start
   ```

2. **Edit the chapter files with your content using Docusaurus Markdown syntax**

3. **For Python/rclpy examples, ensure they are tested in a ROS 2 environment before adding to documentation**

## Build and Deploy

1. **Build the static site:**
   ```bash
   npm run build
   ```

2. **Test the build locally:**
   ```bash
   npm run serve
   ```

3. **Deploy to GitHub Pages following your repository's deployment workflow**

## Content Guidelines

- All examples must use Python and rclpy as specified
- Focus on humanoid robots, not generic mobile robots
- Include clear explanations of ROS 2 concepts like DDS, Nodes, Topics, and Services
- Provide practical examples that readers can reproduce
- Ensure content is accessible to beginner-to-intermediate audience