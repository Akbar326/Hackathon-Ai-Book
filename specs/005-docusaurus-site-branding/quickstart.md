# Quickstart Guide: Docusaurus Site Branding Update

## Overview

This guide provides a quick reference for implementing the Docusaurus site branding changes from "ROS 2 Educational Module" to "Physical AI & Humanoid Robotics".

## Prerequisites

- Node.js and npm installed
- Docusaurus CLI knowledge
- Access to the project repository

## Step-by-Step Implementation

### 1. Create Landing Page
```bash
# Create the pages directory if it doesn't exist
mkdir -p frontend-book/src/pages

# Create the landing page
# (This will be implemented in the tasks phase)
```

### 2. Update Site Configuration
Edit `frontend-book/docusaurus.config.js`:
- Change `title` to "Physical AI & Humanoid Robotics"
- Change `navbar.title` to "Physical AI & Humanoid Robotics"
- Update sidebar label from "Tutorial" to "Text Book"
- Remove blog and GitHub links from navbar items

### 3. Create New Logo
- Create a new logo in `frontend-book/static/img/logo.svg`
- Logo should reflect Physical AI & Humanoid Robotics theme

### 4. Update Navigation Structure
- Modify `frontend-book/sidebars.js` to ensure proper labeling
- Ensure "Text Book" section contains all modules

## Key Files to Modify

1. `frontend-book/docusaurus.config.js` - Site configuration
2. `frontend-book/sidebars.js` - Navigation structure
3. `frontend-book/src/pages/index.js` - Landing page
4. `frontend-book/static/img/logo.svg` - New logo asset

## Verification Steps

1. Start development server: `cd frontend-book && npm run start`
2. Verify homepage shows correct branding
3. Check that "Start Reading" button works
4. Confirm navigation only shows relevant items
5. Ensure all documentation modules are accessible