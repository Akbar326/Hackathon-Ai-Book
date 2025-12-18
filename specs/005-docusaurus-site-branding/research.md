# Research: Docusaurus Site Structure and Branding

## Overview

This research document captures findings about the current Docusaurus site structure and identifies the changes needed to implement the requested branding updates.

## Current State Analysis

### Directory Structure
- **Project location**: `frontend-book/`
- **Configuration**: `frontend-book/docusaurus.config.js`
- **Navigation**: `frontend-book/sidebars.js`
- **Documentation**: `frontend-book/docs/` (contains modules 1-4)
- **Source**: `frontend-book/src/` (contains css directory)
- **Assets**: No dedicated static/public directory found yet

### Current Configuration Details

#### docusaurus.config.js
- **Title**: 'ROS 2 Module - The Robotic Nervous System'
- **Tagline**: 'Educational content for Physical AI & Humanoid Robotics learners'
- **Navbar title**: 'ROS 2 Educational Module'
- **Logo**: 'img/logo.svg' (references non-existent image)
- **Items**:
  - Sidebar with label 'Tutorial' (sidebarId: 'tutorialSidebar')
  - Blog link
  - GitHub link
- **Footer**: Contains links to docs, community, and more sections

#### sidebars.js
- **Sidebar ID**: 'tutorialSidebar'
- **Structure**: Contains 4 modules (1-4) with proper category organization
- **Labels**: Module titles are descriptive and well-structured

### Identified Unknowns and Solutions

#### 1. Landing Page Creation
**Unknown**: No current landing page exists
**Solution**: Create `frontend-book/src/pages/index.js` with Docusaurus-compliant landing page

#### 2. Logo Asset
**Unknown**: Current config references 'img/logo.svg' which doesn't exist
**Solution**: Create new logo matching "Physical AI & Humanoid Robotics" theme and place in `static/img/` directory

#### 3. Branding Text Changes
**Decision**: All branding text needs to be updated:
- Site title: "ROS 2 Module - The Robotic Nervous System" → "Physical AI & Humanoid Robotics"
- Navbar title: "ROS 2 Educational Module" → "Physical AI & Humanoid Robotics"
- Sidebar label: "Tutorial" → "Text Book"

#### 4. Navigation Updates
**Decision**: Remove unwanted navigation items:
- Remove blog link from navbar
- Remove GitHub link from navbar
- Keep theme toggle functionality

## Implementation Approach

### 1. Landing Page
Create `frontend-book/src/pages/index.js` with:
- Main heading: "Physical AI & Humanoid Robotics"
- Introduction text as specified
- "Start Reading" button linking to first module

### 2. Configuration Updates
Update `docusaurus.config.js`:
- Change site title and navbar title
- Update sidebar label from "Tutorial" to "Text Book"
- Remove blog and GitHub links
- Update logo reference

### 3. Assets
Create `frontend-book/static/img/` directory and add new logo

### 4. Navigation
Update `sidebars.js` if needed to ensure proper "Text Book" labeling

## Technical Considerations

- Docusaurus follows convention over configuration
- Static assets go in `static/` directory and are served at `/`
- Pages in `src/pages/` become routes automatically
- Navbar configuration controls top navigation
- Sidebar configuration controls documentation navigation

## Risks and Mitigations

1. **Asset Path Issues**: Ensure logo path is correct after creation
2. **Navigation Breakage**: Test all links after navbar changes
3. **Module Access**: Ensure all existing content remains accessible under new structure