# Quickstart: Digital Twin Module Development

## Prerequisites

- Node.js 18.x or higher
- npm or yarn package manager
- Git for version control
- Basic knowledge of Markdown syntax
- Understanding of ROS 2 fundamentals (from Module 1)

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
   - Update the `title` and `tagline` to reflect the Digital Twin module
   - Update the `url` and `baseUrl` for deployment
   - Configure the `organizationName` and `projectName` for GitHub Pages

2. **Set up the sidebar navigation** in `sidebars.js`:
   ```javascript
   module.exports = {
     tutorial: [
       'intro',
       {
         type: 'category',
         label: 'Module 1 - The Robotic Nervous System',
         items: [
           'module-1/intro',
           'module-1/ros2-communication',
           'module-1/ai-to-robot-bridge',
           'module-1/summary',
         ],
       },
       {
         type: 'category',
         label: 'Module 2 - The Digital Twin',
         items: [
           'module-2/gazebo-simulation',
           'module-2/unity-rendering',
           'module-2/sensors-simulation',
         ],
       },
     ],
   };
   ```

## Create Module Content

1. **Create the module directory**:
   ```bash
   mkdir docs/module-2
   ```

2. **Create the three required chapters**:
   - `docs/module-2/gazebo-simulation.md`
   - `docs/module-2/unity-rendering.md`
   - `docs/module-2/sensors-simulation.md`

3. **Add frontmatter to each chapter**:
   ```markdown
   ---
   id: gazebo-simulation
   title: Simulating Physical Environments in Gazebo
   sidebar_label: Gazebo Simulation
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

- Use clear, educational language appropriate for students with ROS 2 basics knowledge
- Include practical examples and code snippets where relevant
- Structure content with clear headings and subheadings
- Use Docusaurus features like admonitions for important notes
- Ensure all links and references are valid

## Next Steps

1. Complete the content for all three chapters
2. Review and test the navigation
3. Validate all code examples
4. Test deployment to GitHub Pages