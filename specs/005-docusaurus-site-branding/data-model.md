# Data Model: Docusaurus Site Structure and Branding

## Overview

This project primarily involves UI and configuration changes rather than data model modifications. The following entities represent the key configuration elements that will be updated.

## Entities

### Site Configuration
- **Name**: Site title and branding information
- **Attributes**:
  - title: string (e.g., "Physical AI & Humanoid Robotics")
  - tagline: string (e.g., "Educational content for Physical AI & Humanoid Robotics learners")
  - navbarTitle: string (e.g., "Physical AI & Humanoid Robotics")
- **Relationships**: Configures the overall site identity

### Navigation Structure
- **Name**: Navigation menu and sidebar organization
- **Attributes**:
  - sidebarLabel: string (e.g., "Text Book" replacing "Tutorial")
  - navItems: array of navigation items
  - excludedItems: array of items to remove (e.g., blog, GitHub)
- **Relationships**: Controls user navigation paths

### Landing Page Content
- **Name**: Homepage content structure
- **Attributes**:
  - heading: string ("Physical AI & Humanoid Robotics")
  - introduction: string (long-form description)
  - callToAction: object with label and destination
- **Relationships**: Serves as entry point to documentation

### Asset Configuration
- **Name**: Static assets (logo, images)
- **Attributes**:
  - logoPath: string (path to logo file)
  - logoAltText: string (accessibility text)
- **Relationships**: Provides visual branding elements