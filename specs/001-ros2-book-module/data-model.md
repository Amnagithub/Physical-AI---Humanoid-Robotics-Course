# Data Model: ROS 2 Book Module

## Content Entities

### Module
- **name**: String - The module title ("The Robotic Nervous System (ROS 2)")
- **description**: String - Brief description of the module's purpose
- **chapters**: Array<Chapter> - List of chapters in the module
- **target_audience**: String - Beginner-to-intermediate robotics audience
- **prerequisites**: Array<String> - Prerequisites needed to understand the content

### Chapter
- **id**: String - Unique identifier for the chapter
- **title**: String - Chapter title
- **description**: String - Brief description of the chapter content
- **sections**: Array<Section> - List of sections in the chapter
- **learning_objectives**: Array<String> - What the learner should understand after reading
- **practical_examples**: Array<Example> - Hands-on examples included in the chapter

### Section
- **title**: String - Section title
- **content**: String - Markdown content of the section
- **type**: Enum - Type of content (theoretical, practical, example, exercise)
- **dependencies**: Array<String> - Other sections this section depends on

### Example
- **title**: String - Title of the example
- **description**: String - Brief description of what the example demonstrates
- **code**: String - Python/rclpy code for the example
- **expected_output**: String - What the learner should expect to see
- **robot_model**: String - Which robot model the example applies to (for humanoid focus)

## Content Relationships

- Module contains multiple Chapters
- Chapter contains multiple Sections
- Chapter contains multiple Examples
- Section may reference other Sections as dependencies

## Validation Rules

- Each Chapter must have a unique title within the Module
- Each Chapter must have at least one learning objective
- Each Example must have valid Python/rclpy code syntax
- Each Section must have content of at least 100 words (or be marked as intentionally brief)
- Module must have exactly 3 Chapters as specified in the requirements

## State Transitions

- Content starts as "Draft"
- Content moves to "Reviewed" after peer review
- Content moves to "Published" when included in the live documentation