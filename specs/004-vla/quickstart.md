# Quickstart: Vision-Language-Action Module Development

## Prerequisites

- Node.js 18.x or higher
- npm or yarn package manager
- Git for version control
- Basic knowledge of Markdown syntax
- Understanding of ROS 2 fundamentals and simulation concepts (from Modules 1-3)
- Basic knowledge of AI/ML concepts

## Setup Docusaurus Environment

1. **If Docusaurus is not already installed**, install it:
   ```bash
   npx create-docusaurus@latest website classic
   ```

2. **Navigate to your Docusaurus project directory**:
   ```bash
   cd frontend-book  # or your Docusaurus project directory
   ```

3. **Install required dependencies** (if not already installed):
   ```bash
   npm install
   ```

## Configure Your Site

1. **Edit the site configuration** in `docusaurus.config.js`:
   - Update the `title` and `tagline` to reflect the Vision-Language-Action module
   - Update the `url` and `baseUrl` for deployment
   - Configure the `organizationName` and `projectName` for GitHub Pages

2. **Set up the sidebar navigation** in `sidebars.js`:
   ```javascript
   module.exports = {
     tutorial: [
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
           'module-4/capstone-autonomous-humanoid',
         ],
       },
     ],
   };
   ```

## Create Module Content

1. **Create the module directory**:
   ```bash
   mkdir docs/module-4
   ```

2. **Create the three required chapters**:
   - `docs/module-4/voice-to-action.md`
   - `docs/module-4/language-planning.md`
   - `docs/module-4/capstone-autonomous-humanoid.md`

3. **Add frontmatter to each chapter**:
   ```markdown
   ---
   id: voice-to-action
   title: Voice-to-Action Interfaces
   sidebar_label: Voice-to-Action
   ---

   Content goes here...
   ```

## Local Development

1. **Start the development server**:
   ```bash
   npm start
   ```

2. **Open your browser** to `http://localhost:3000` to view your site

3. **Edit the Markdown files** in the `docs` directory to see live updates

## Build and Deployment

1. **Build the static site**:
   ```bash
   npm run build
   ```

2. **Test the build locally**:
   ```bash
   npm run serve
   ```

3. **Deploy to GitHub Pages** (if configured):
   ```bash
   npm run deploy
   ```

## Content Guidelines

- Use clear, educational language appropriate for students with ROS 2, simulation, and AI/ML knowledge
- Include practical examples and code snippets where relevant
- Structure content with clear headings and subheadings
- Use Docusaurus features like admonitions for important notes
- Ensure all links and references are valid

## Next Steps

1. Complete the content for all three chapters
2. Review and test the navigation
3. Validate all code examples
4. Test deployment to GitHub Pages