---
id: 1
title: fix-docusaurus-site-branding
stage: spec
date: 2025-12-18
surface: agent
model: haiku
feature: docusaurus-site-branding
branch: 005-docusaurus-site-branding
user: Claude user
command: /sp.specify
labels: ["docusaurus", "branding", "ui", "spec"]
links:
  spec: specs/005-docusaurus-site-branding/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/005-docusaurus-site-branding/spec.md
 - specs/005-docusaurus-site-branding/checklists/requirements.md
tests:
 - specs/005-docusaurus-site-branding/checklists/requirements.md
---

## Prompt

You are working inside a Spec-Kit Plus project using Claude Code CLI.

Goal:
Fix Docusaurus site structure and branding for an AI textbook project.

Project Name Update:
- Change site name from "ROS 2 Educational Module"
- New name: "Physical AI & Humanoid Robotics"
- Subtitle/tagline should reflect embodied intelligence and physical AI systems

Landing Page (Home Page):
- Create a proper Docusaurus landing page (no Page Not Found)
- Landing page must include:
  - Main heading: "Physical AI & Humanoid Robotics"
  - Short introduction based on the following text:

    "Step into the world where AI leaves the screen and enters reality. This book introduces Physical AI—intelligent systems that perceive, reason, and act in the real world.You'll learn how digital intelligence connects to a physical body through humanoid robots, explore simulation techniques, and understand how perception, language, and action combine for natural human-robot interaction.
    By the end, you'll be able to design, simulate, and and control humanoid robots using ROS 2, Gazebo, and NVIDIA Isaac, bringing AI from software into autonomous, embodied agents."

- Add a primary CTA button:
  - Label: "Start Reading"
  - On click: navigates directly to the Text Book (docs) first module

Docs / Text Book Changes:
- Rename "Tutorial" section to "Text Book"
- Ensure all modules and chapters appear under Text Book
- Set Module 1 as the default entry when clicking "Start Reading"

Navbar Updates:
- Remove "Blog" from navbar
- Remove GitHub icon/link from navbar
- Keep only relevant items (Text Book, theme toggle if needed)

Logo:
- Replace existing logo with a new logo concept matching:
  - Physical AI
  - Humanoid robotics
  - Robotics + intelligence theme
- Use a clean, modern, technical style
- Logo text should align with:
  "Physical AI & Humanoid Robotics"

Technical Constraints:
- Use Docusaurus best practices
- Update docusaurus.config.js and sidebars accordingly
- All documentation files must remain Markdown (.md)
- Do not break existing module content

Deliverables:
- Fixed landing page
- Renamed Text Book section
- Updated navbar
- Updated site title
- New logo asset integrated

## Response snapshot

# Feature Specification: Docusaurus Site Structure and Branding

**Feature Branch**: `005-docusaurus-site-branding`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "Fix Docusaurus site structure and branding for an AI textbook project."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Visit Landing Page (Priority: P1)

As a visitor to the website, I want to see a proper landing page with the correct branding so that I understand what the site is about and can begin reading the textbook.

**Why this priority**: This is the first impression users have of the site and sets the foundation for the entire user experience.

**Independent Test**: User visits the homepage and sees the "Physical AI & Humanoid Robotics" heading with proper introduction text and a "Start Reading" button that leads to the textbook.

**Acceptance Scenarios**:

1. **Given** a user navigates to the homepage, **When** they land on the page, **Then** they see the heading "Physical AI & Humanoid Robotics" with the introduction text and a "Start Reading" button
2. **Given** a user is on the homepage, **When** they click the "Start Reading" button, **Then** they are taken to the first module of the textbook

---

### User Story 2 - Navigate Using Updated Navbar (Priority: P1)

As a user browsing the site, I want to see a clean navigation bar without irrelevant links so that I can focus on the textbook content.

**Why this priority**: Clean navigation improves user focus and reduces confusion about the site's purpose.

**Independent Test**: User sees a navigation bar with only relevant items (Text Book, theme toggle) and no Blog or GitHub links.

**Acceptance Scenarios**:

1. **Given** a user views any page on the site, **When** they look at the navigation bar, **Then** they see only "Text Book" and theme toggle options without Blog or GitHub links

---

### User Story 3 - Experience Consistent Branding (Priority: P2)

As a user reading the textbook, I want to see consistent branding throughout the site so that I understand I'm in the right place and the site looks professional.

**Why this priority**: Consistent branding builds trust and ensures users know they're in the right place.

**Independent Test**: User sees the updated site name "Physical AI & Humanoid Robotics" and matching logo throughout the site.

**Acceptance Scenarios**:

1. **Given** a user navigates through different pages of the site, **When** they view the header/logo area, **Then** they see consistent branding with the new name and logo

---

### User Story 4 - Access Text Book Content (Priority: P1)

As a student reading the textbook, I want to access the content organized under "Text Book" instead of "Tutorial" so that the categorization makes sense.

**Why this priority**: Proper categorization helps users understand the content structure and find what they're looking for.

**Independent Test**: User can navigate to the Text Book section and see all modules and chapters properly organized under this new section.

**Acceptance Scenarios**:

1. **Given** a user clicks on "Text Book" in the navigation, **When** they view the sidebar, **Then** they see all modules and chapters listed under the Text Book category
2. **Given** a user is viewing the first module, **When** they navigate from the homepage, **Then** they land on Module 1 of the textbook

---

### Edge Cases

- What happens when a user bookmarks a link to the old "Tutorial" section?
- How does the system handle direct navigation to specific modules after the reorganization?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display "Physical AI & Humanoid Robotics" as the site title and brand
- **FR-002**: System MUST show a landing/homepage with the heading "Physical AI & Humanoid Robotics" and the specified introduction text
- **FR-003**: System MUST include a "Start Reading" button on the homepage that navigates directly to the first module of the textbook
- **FR-004**: System MUST rename the "Tutorial" section to "Text Book" in navigation and sidebar
- **FR-005**: System MUST remove "Blog" from the navigation bar
- **FR-006**: System MUST remove GitHub icon/link from the navigation bar
- **FR-007**: System MUST update the site logo to reflect Physical AI & Humanoid Robotics theme
- **FR-008**: System MUST ensure all existing documentation files remain accessible under the new structure
- **FR-009**: System MUST maintain all existing module content while reorganizing under "Text Book" section

### Key Entities

- **Site Branding**: Represents the visual identity of the website including title, logo, and tagline
- **Navigation Structure**: Represents the organization of site sections and their accessibility to users
- **Documentation Content**: Represents the educational materials and modules that make up the textbook

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of visitors to the homepage see the correct branding ("Physical AI & Humanoid Robotics") and introduction text
- **SC-002**: Users can access the first module of the textbook within 2 clicks from the homepage (clicking "Start Reading")
- **SC-003**: Navigation bar contains only relevant items (Text Book, theme toggle) with no Blog or GitHub links
- **SC-004**: All existing documentation content remains accessible and functional under the new "Text Book" organization
- **SC-005**: Site performance and load times remain unchanged after branding updates

## Outcome

- ✅ Impact: Created comprehensive specification for Docusaurus site rebranding and structural changes
- 🧪 Tests: Created requirements checklist to validate specification quality
- 📁 Files: Created spec.md and requirements checklist in new feature directory
- 🔁 Next prompts: Ready for planning phase with `/sp.plan`
- 🧠 Reflection: Specification covers all user requirements with clear acceptance criteria and measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A