# Data Model: Vision-Language-Action Module Content Structure

## Content Entities

### LearningModule
- **name**: String (e.g., "Module 4 - Vision-Language-Action")
- **description**: String (overview of the module's purpose)
- **learningObjectives**: Array of String (what students will learn)
- **targetAudience**: String (students with ROS 2, simulation, and AI/ML knowledge)
- **chapters**: Array of Chapter references
- **prerequisites**: Array of String (required knowledge, e.g., "ROS 2 fundamentals, simulation basics, AI/ML concepts")

### Chapter
- **id**: String (unique identifier like "voice-to-action", "language-planning", "capstone-autonomous-humanoid")
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
- **language**: String (e.g., "python", "bash", "yaml")
- **code**: String (the actual code snippet)
- **description**: String (what the example demonstrates)
- **explanation**: String (step-by-step explanation of the code)

### VLAPipeline
- **name**: String (e.g., "Voice-to-Action Pipeline", "Language Planning Pipeline")
- **components**: Array of String (the different components in the pipeline)
- **inputs**: Array of String (types of input the pipeline accepts)
- **outputs**: Array of String (types of output the pipeline produces)
- **integrationPoints**: Array of String (how it connects with other systems)

### IntentClassifier
- **name**: String (name of the intent classification system)
- **inputType**: String (type of input, e.g., "transcribed speech", "natural language")
- **classificationMethod**: String (method used for classification)
- **outputFormat**: String (format of the structured intent output)
- **accuracyMetrics**: Object (metrics for measuring classification performance)

### LLMPlanner
- **name**: String (name of the LLM planning system)
- **modelType**: String (type of LLM used)
- **taskDecomposition**: String (approach to breaking down tasks)
- **safetyMechanisms**: Array of String (mechanisms to ensure safe plan generation)
- **groundingMethods**: Array of String (methods to ground plans in reality)

## Relationships

- LearningModule contains multiple Chapters
- Chapter contains multiple ContentSections
- ContentSection may reference multiple CodeExamples
- ContentSection may define multiple VLAPipelines
- VLAPipeline may be referenced by multiple ContentSections
- IntentClassifier may be part of a VLAPipeline
- LLMPlanner may be part of a VLAPipeline

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