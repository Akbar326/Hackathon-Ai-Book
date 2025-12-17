# Data Model: Digital Twin Module Content Structure

## Content Entities

### LearningModule
- **name**: String (e.g., "Module 2 - The Digital Twin")
- **description**: String (overview of the module's purpose)
- **learningObjectives**: Array of String (what students will learn)
- **targetAudience**: String (students with ROS 2 basics knowledge)
- **chapters**: Array of Chapter references
- **prerequisites**: Array of String (required knowledge, e.g., "ROS 2 fundamentals")

### Chapter
- **id**: String (unique identifier like "gazebo-simulation", "unity-rendering", "sensors-simulation")
- **title**: String (display title)
- **content**: String (Markdown content)
- **learningObjectives**: Array of String (specific objectives for this chapter)
- **duration**: Number (estimated time to complete in minutes)
- **prerequisites**: Array of String (knowledge needed before this chapter)
- **nextChapter**: Chapter reference (optional, for progression)

### ContentSection
- **title**: String (section heading)
- **content**: String (Markdown content for the section)
- **type**: Enum ("text", "code", "diagram", "exercise", "example")
- **difficulty**: Enum ("beginner", "intermediate", "advanced")

### CodeExample
- **language**: String (e.g., "python", "bash", "xml")
- **code**: String (the actual code snippet)
- **description**: String (what the example demonstrates)
- **explanation**: String (step-by-step explanation of the code)

### SimulationConcept
- **name**: String (e.g., "Physics Simulation", "Sensor Fusion", "Rendering")
- **definition**: String (clear definition of the concept)
- **purpose**: String (why this concept exists and what problem it solves)
- **examples**: Array of CodeExample references
- **relatedConcepts**: Array of SimulationConcept references

### SensorType
- **name**: String (e.g., "LiDAR", "Depth Camera", "IMU")
- **properties**: Object (characteristics like range, resolution, update rate)
- **simulationMethod**: String (how the sensor is simulated in virtual environments)
- **dataFormat**: String (format of the simulated data output)
- **useCases**: Array of String (typical applications in robotics)

## Relationships

- LearningModule contains multiple Chapters
- Chapter contains multiple ContentSections
- ContentSection may reference multiple CodeExamples
- ContentSection may define multiple SimulationConcepts
- SimulationConcept may be related to multiple other SimulationConcepts
- SensorType may be referenced by multiple ContentSections

## Validation Rules

- Each LearningModule must have at least one Chapter
- Each Chapter must have a unique ID within the module
- Each Chapter must have a non-empty title and content
- ContentSections of type "exercise" must have a solution or guidance
- All code examples must be valid syntax for the specified language
- Learning objectives must be measurable and specific

## State Transitions

- Content starts in "draft" state
- Content moves to "review" state after initial writing
- Content moves to "approved" state after review and validation
- Content can move back to "draft" for revisions