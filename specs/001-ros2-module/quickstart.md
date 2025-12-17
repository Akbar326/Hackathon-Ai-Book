# Quickstart: ROS 2 Module Development

## Prerequisites

- Node.js 18.x or higher
- npm or yarn package manager
- Git for version control
- Basic knowledge of Markdown syntax

## Setup Docusaurus Environment

1. **Install Docusaurus CLI** (if not already installed):
   ```bash
   npm install -g @docusaurus/core@latest
   ```

2. **Create a new Docusaurus project** in your repository root:
   ```bash
   npx create-docusaurus@latest website classic
   ```

3. **Navigate to your project directory**:
   ```bash
   cd website
   ```

4. **Install required dependencies**:
   ```bash
   npm install
   ```

## Configure Your Site

1. **Edit the site configuration** in `docusaurus.config.js`:
   - Update the `title` and `tagline` to reflect the ROS 2 module
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
         ],
       },
     ],
   };
   ```

## Create Module Content

1. **Create the module directory**:
   ```bash
   mkdir docs/module-1
   ```

2. **Create the three required chapters**:
   - `docs/module-1/intro.md`
   - `docs/module-1/ros2-communication.md`
   - `docs/module-1/ai-to-robot-bridge.md`

3. **Add frontmatter to each chapter**:
   ```markdown
   ---
   id: intro
   title: Introduction to ROS 2 for Physical AI
   sidebar_label: Introduction
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

- Use clear, educational language appropriate for students with basic Python/AI knowledge
- Include practical examples and code snippets where relevant
- Structure content with clear headings and subheadings
- Use Docusaurus features like admonitions for important notes
- Ensure all links and references are valid

## Next Steps

1. Complete the content for all three chapters
2. Review and test the navigation
3. Validate all code examples
4. Test deployment to GitHub Pages