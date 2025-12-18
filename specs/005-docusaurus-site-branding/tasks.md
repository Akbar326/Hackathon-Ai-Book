---
description: "Task list for Docusaurus site structure and branding implementation"
---

# Tasks: Docusaurus Site Structure and Branding

**Input**: Design documents from `/specs/005-docusaurus-site-branding/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: No explicit test requirements in feature specification - tests are not included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `frontend-book/` at repository root
- Paths shown below follow the project structure from plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create static directory for assets in frontend-book/static/
- [X] T002 Create pages directory for landing page in frontend-book/src/pages/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T003 Create new logo asset in frontend-book/static/img/logo.svg
- [X] T004 Update docusaurus.config.js to reference new site title and branding

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Visit Landing Page (Priority: P1) 🎯 MVP

**Goal**: Create a proper landing page with the correct branding so that users understand what the site is about and can begin reading the textbook.

**Independent Test**: User visits the homepage and sees the "Physical AI & Humanoid Robotics" heading with proper introduction text and a prominent "Start Reading" button that leads to the textbook.

### Implementation for User Story 1

- [X] T005 [US1] Create landing page component in frontend-book/src/pages/index.js with heading "Physical AI & Humanoid Robotics"
- [X] T006 [US1] Add introduction text to landing page in frontend-book/src/pages/index.js as specified in requirements
- [X] T007 [US1] Add "Start Reading" button to landing page in frontend-book/src/pages/index.js that links to first module
- [X] T008 [US1] Style landing page to match Docusaurus conventions in frontend-book/src/pages/index.js

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Navigate Using Updated Navbar (Priority: P1)

**Goal**: Update navigation to show a clean navigation bar without irrelevant links so that users can focus on the textbook content.

**Independent Test**: User sees a navigation bar with only relevant items (Text Book, theme toggle) and no Blog or GitHub links.

### Implementation for User Story 2

- [X] T009 [US2] Update navbar configuration in frontend-book/docusaurus.config.js to remove blog link
- [X] T010 [US2] Update navbar configuration in frontend-book/docusaurus.config.js to remove GitHub link
- [X] T011 [US2] Verify navbar only contains relevant items in frontend-book/docusaurus.config.js

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Experience Consistent Branding (Priority: P2)

**Goal**: Ensure consistent branding throughout the site so that users understand they're in the right place and the site looks professional.

**Independent Test**: User sees the updated site name "Physical AI & Humanoid Robotics" and matching logo throughout the site.

### Implementation for User Story 3

- [X] T012 [US3] Update site title in frontend-book/docusaurus.config.js to "Physical AI & Humanoid Robotics"
- [X] T013 [US3] Update navbar title in frontend-book/docusaurus.config.js to "Physical AI & Humanoid Robotics"
- [X] T014 [US3] Update site tagline in frontend-book/docusaurus.config.js to reflect Physical AI & Humanoid Robotics
- [X] T015 [US3] Verify logo displays correctly in navbar in frontend-book/docusaurus.config.js

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Access Text Book Content (Priority: P1)

**Goal**: Allow users to access the content organized under "Text Book" instead of "Tutorial" so that the categorization makes sense.

**Independent Test**: User can navigate to the Text Book section and see all modules and chapters properly organized under this new section.

### Implementation for User Story 4

- [X] T016 [US4] Update sidebar label from "Tutorial" to "Text Book" in frontend-book/docusaurus.config.js
- [X] T017 [US4] Update sidebar configuration in frontend-book/sidebars.js to ensure proper "Text Book" labeling
- [X] T018 [US4] Verify all modules appear under "Text Book" section in frontend-book/sidebars.js
- [X] T019 [US4] Ensure "Start Reading" button from landing page navigates to Module 1 of Text Book

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T020 Update footer links to reflect new branding in frontend-book/docusaurus.config.js
- [X] T021 Verify all existing documentation content remains accessible under new structure
- [X] T022 Test site locally to ensure all changes work together properly
- [X] T023 Run quickstart.md validation steps

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: All User Stories

```bash
# Once foundational phase is complete, all user stories can proceed in parallel:
# Developer A: User Story 1 - Landing page implementation
# Developer B: User Story 2 - Navigation updates
# Developer C: User Story 3 - Branding consistency
# Developer D: User Story 4 - Text Book content access
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 4 - All Priority P1)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 - Landing page
4. Complete Phase 4: User Story 2 - Navigation
5. Complete Phase 6: User Story 4 - Text Book access
6. **STOP and VALIDATE**: Test core functionality independently
7. Deploy/demo if ready

### Add Higher Value Features

8. Complete Phase 5: User Story 3 - Consistent branding
9. Complete Phase 7: Polish and validation

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Stories 1, 2, 4 → Core functionality → Test independently → Deploy/Demo (MVP!)
3. Add User Story 3 → Consistent branding → Test independently → Deploy/Demo
4. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 4 (also P1)
   - Developer D: User Story 3 (P2)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence