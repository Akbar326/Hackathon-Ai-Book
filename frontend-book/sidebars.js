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
    {
      type: 'category',
      label: 'Module 3 - The AI-Robot Brain',
      items: [
        'module-3/isaac-sim',
        'module-3/isaac-ros',
        'module-3/nav2-navigation',
        'module-3/summary'
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action',
      items: [
        'module-4/voice-to-action',
        'module-4/language-planning',
        'module-4/capstone-autonomous-humanoid'
      ],
    },
  ],
};

module.exports = sidebars;