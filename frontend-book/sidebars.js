// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'module-1/intro',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System',
      items: [
        'module-1/ros2-communication',
        'module-1/ai-to-robot-bridge',
        'module-1/summary'
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - The Digital Twin',
      items: [
        'module-2/gazebo-simulation',
        'module-2/unity-rendering',
        'module-2/sensors-simulation',
        'module-2/summary'
      ],
    },
  ],
};

module.exports = sidebars;