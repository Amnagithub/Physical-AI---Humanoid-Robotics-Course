// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
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
      ],
    },
  ],
};

module.exports = sidebars;