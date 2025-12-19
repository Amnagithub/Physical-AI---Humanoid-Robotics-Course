# Data Model: Digital Twin Book Module

## Module Entity
- **Name**: Digital Twin Module (Gazebo & Unity)
- **Description**: Module 2 of Physical AI & Humanoid Robotics course
- **Content Type**: Educational documentation
- **Format**: Docusaurus Markdown
- **Target Audience**: Robotics students and developers
- **Learning Objectives**: Understand digital twins, simulate humanoid robots, configure sensors

## Chapter Entity
- **Chapter ID**: Unique identifier (e.g., ch-001-gazebo)
- **Title**: Descriptive title of the chapter
- **Description**: Brief overview of chapter content
- **Learning Goals**: Specific skills/knowledge to be acquired
- **Prerequisites**: Required knowledge or setup before starting
- **Duration**: Estimated time to complete
- **Exercises Count**: Number of practical exercises included
- **Related Chapters**: Cross-references to other chapters

## Content Page Entity
- **Page ID**: Unique identifier for each page
- **Title**: Page title displayed in navigation
- **Content Type**: Overview, tutorial, reference, exercise
- **Body Content**: Markdown content of the page
- **Metadata**: Author, creation date, last updated
- **Navigation Order**: Position in the chapter sequence
- **Parent Chapter**: Reference to the containing chapter
- **Related Pages**: Links to related content

## Exercise Entity
- **Exercise ID**: Unique identifier for each exercise
- **Title**: Brief description of the exercise
- **Objective**: What the student should achieve
- **Prerequisites**: Knowledge or setup required
- **Steps**: Sequential instructions for completion
- **Expected Outcome**: What success looks like
- **Difficulty Level**: Beginner, Intermediate, Advanced
- **Estimated Time**: Time required to complete
- **Solution**: Reference solution or expected results

## Configuration Guide Entity
- **Guide ID**: Unique identifier for configuration guides
- **Target Tool**: Gazebo, Unity, specific sensor type
- **Purpose**: What the configuration accomplishes
- **Parameters**: Key configuration values and explanations
- **Default Values**: Standard settings for typical use
- **Customization Options**: How to modify for specific needs
- **Troubleshooting**: Common issues and solutions

## Sensor Data Entity
- **Sensor Type**: LiDAR, depth camera, IMU
- **Data Format**: Structure of the simulated data
- **Parameters**: Configuration options for simulation
- **Interpretation Guide**: How to understand the output
- **Sample Data**: Example outputs for reference
- **Use Cases**: Common applications of the sensor data
- **Limitations**: Known constraints in simulation

## Relationship Mapping

### Module-Chapter Relationship
- One Module contains Many Chapters
- Each Chapter belongs to exactly One Module
- Chapters have sequential order within Module

### Chapter-Content Page Relationship
- One Chapter contains Many Content Pages
- Each Content Page belongs to exactly One Chapter
- Content Pages have sequential order within Chapter

### Chapter-Exercise Relationship
- One Chapter contains Many Exercises
- Each Exercise belongs to exactly One Chapter
- Exercises may reference multiple Content Pages

### Content Page-Configuration Guide Relationship
- One Content Page may reference Many Configuration Guides
- One Configuration Guide may be referenced by Many Content Pages
- Many-to-many relationship through content links

## Validation Rules

### Module Validation
- Module must have a unique title
- Module must contain at least one chapter
- Module learning objectives must align with course goals

### Chapter Validation
- Chapter must have a unique title within the module
- Chapter must have at least one content page
- Chapter learning goals must be measurable
- Chapter duration must be estimated realistically

### Content Page Validation
- Page title must be unique within chapter
- Content must be in valid Markdown format
- All referenced images and links must exist
- Page must have clear learning objective

### Exercise Validation
- Exercise must have a clear objective
- Exercise steps must be complete and testable
- Expected outcome must be verifiable
- Difficulty level must match content complexity

## State Transitions

### Content Creation Workflow
1. **Draft**: Initial content creation
2. **Review**: Content review by subject matter expert
3. **Validation**: Technical accuracy verification
4. **Published**: Content available to students
5. **Archived**: Content deprecated for updates

### Exercise Status
1. **Design**: Exercise concept and objectives defined
2. **Development**: Exercise content and solution created
3. **Testing**: Exercise validated with actual tools
4. **Approved**: Exercise ready for publication
5. **Active**: Exercise available to students