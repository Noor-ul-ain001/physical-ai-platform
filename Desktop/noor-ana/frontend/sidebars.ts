import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1/week-1-introduction',
        'module-1/week-2-nodes-topics',
        'module-1/week-3-services-actions',
        'module-1/week-4-tf-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      link: {type: 'doc', id: 'intro'}, // Placeholder link
      items: [
        // TODO: Add simulation content
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Sim',
      link: {type: 'doc', id: 'intro'}, // Placeholder link
      items: [
        // TODO: Add Isaac Sim content
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Models',
      link: {type: 'doc', id: 'intro'}, // Placeholder link
      items: [
        // TODO: Add VLA Models content
      ],
    },
  ],
};

export default sidebars;