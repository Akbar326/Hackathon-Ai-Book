// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System',
      items: [
        'module-1/intro',
        'module-1/ros2-communication',
        'module-1/ai-to-robot-bridge',
      ],
    },
  ],
};

module.exports = sidebars;