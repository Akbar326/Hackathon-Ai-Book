# Implementation Plan: Docusaurus Site Structure and Branding

**Branch**: `005-docusaurus-site-branding` | **Date**: 2025-12-18 | **Spec**: specs/005-docusaurus-site-branding/spec.md
**Input**: Feature specification from `/specs/[005-docusaurus-site-branding]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Update the Docusaurus site for the AI textbook project to change branding from "ROS 2 Educational Module" to "Physical AI & Humanoid Robotics", create a proper landing page with introduction text and "Start Reading" button, rename "Tutorial" to "Text Book" section, update navigation, and implement a new logo that matches the Physical AI & Humanoid Robotics theme.

## Technical Context

**Language/Version**: JavaScript/TypeScript, Node.js LTS
**Primary Dependencies**: Docusaurus 2.x, React, Node.js
**Storage**: N/A (static site)
**Testing**: N/A (static site)
**Target Platform**: Web browser (static site hosting)
**Project Type**: Web
**Performance Goals**: Fast loading, SEO optimized
**Constraints**: All existing documentation files must remain accessible, Docusaurus best practices must be followed
**Scale/Scope**: Educational textbook website

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- Spec-Driven Development First: ✅ Plan based on established specification
- Accuracy and Zero Hallucination: N/A (UI/branding change)
- Clear Educational Writing: ✅ Maintaining educational content while improving UX
- Reproducible and Modular Architecture: ✅ Following Docusaurus conventions
- Technology Stack Standards: ✅ Using Docusaurus as specified in constitution
- Compliance and Documentation: ✅ Following established patterns

## Project Structure

### Documentation (this feature)

```text
specs/005-docusaurus-site-branding/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── pages/               # Landing page (index.js)
├── components/          # Custom components if needed
└── css/                 # Custom styles if needed

docs/                    # Documentation files (textbook content)
├── 001-ros2-module/     # Module 1 content
├── 002-digital-twin/    # Module 2 content
├── 003-ai-robot-brain/  # Module 3 content
└── 004-vla/             # Module 4 content

static/                  # Static assets (logo, images)
├── img/                 # Images including new logo

docusaurus.config.js     # Site configuration
sidebars.js              # Navigation configuration
package.json             # Dependencies
```

**Structure Decision**: Using standard Docusaurus project structure with a single static site. The site will have a custom landing page, updated configuration for branding, and reorganized navigation structure.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|