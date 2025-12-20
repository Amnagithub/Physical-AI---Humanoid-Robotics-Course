// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Course Foundations',
      items: [
        'course/why-physical-ai-matters',
        'course/learning-outcomes',
        'course/weekly-breakdown',
        'course/hardware-requirements',
        'course/lab-architecture'
      ],
    },
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'ROS 2: The Robotic Nervous System',
          items: [
            'modules/ros2-nervous-system/intro',
            'modules/ros2-nervous-system/index',
            'modules/ros2-nervous-system/chapter1-intro',
            'modules/ros2-nervous-system/chapter2-communication',
            'modules/ros2-nervous-system/chapter3-humanoid-control',
            'modules/ros2-nervous-system/glossary',
            'modules/ros2-nervous-system/troubleshooting'
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            'modules/digital-twin/index',
            {
              type: 'category',
              label: 'Chapter 1: Gazebo Physics Simulation',
              items: [
                'modules/digital-twin/chapter-1-gazebo-physics/index',
                'modules/digital-twin/chapter-1-gazebo-physics/setup',
                'modules/digital-twin/chapter-1-gazebo-physics/physics-concepts',
                'modules/digital-twin/chapter-1-gazebo-physics/humanoid-dynamics',
                'modules/digital-twin/chapter-1-gazebo-physics/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Unity Environments and Interaction',
              items: [
                'modules/digital-twin/chapter-2-unity-env/index',
                'modules/digital-twin/chapter-2-unity-env/unity-setup',
                'modules/digital-twin/chapter-2-unity-env/rendering',
                'modules/digital-twin/chapter-2-unity-env/interaction',
                'modules/digital-twin/chapter-2-unity-env/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Humanoid Sensor Simulation',
              items: [
                'modules/digital-twin/chapter-3-sensor-sim/index',
                'modules/digital-twin/chapter-3-sensor-sim/sensor-types',
                'modules/digital-twin/chapter-3-sensor-sim/config-guide',
                'modules/digital-twin/chapter-3-sensor-sim/data-interpretation',
                'modules/digital-twin/chapter-3-sensor-sim/exercises'
              ],
            }
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'modules/module-3/index',
            {
              type: 'category',
              label: 'Chapter 1: Photorealistic Simulation and Synthetic Data Generation',
              items: [
                'modules/module-3/chapter-1-simulation',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Hardware-Accelerated VSLAM and Navigation',
              items: [
                'modules/module-3/chapter-2-vslam-navigation',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Path Planning and Bipedal Humanoid Movement',
              items: [
                'modules/module-3/chapter-3-path-planning',
              ],
            }
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          items: [
            'modules/module-4/index',
            {
              type: 'category',
              label: 'Chapter 1: Voice Control and Speech Recognition',
              items: [
                'modules/module-4/chapter-1-voice-control',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Cognitive Planning with Large Language Models',
              items: [
                'modules/module-4/chapter-2-llm-planning',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Autonomous Humanoid Capstone Project',
              items: [
                'modules/module-4/chapter-3-autonomous-capstone',
              ],
            }
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;