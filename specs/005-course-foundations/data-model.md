# Data Model: Course Foundations

## Document Structure

### Base Document Schema
All course foundation documents follow this structure:

```yaml
---
sidebar_label: '<Document Title>'
title: '<Document Title>'
---
```

```markdown
# <Document Title>

## Section 1
Content for section 1

## Section 2
Content for section 2

...
```

### Document Types

#### 1. Philosophy Document
- **File**: `why-physical-ai-matters.md`
- **Purpose**: Course philosophy and motivation
- **Key Elements**:
  - Shift from digital AI to embodied intelligence
  - Limitations of digital-only AI
  - Emergence of embodied intelligence
  - Applications and impact
  - Path forward

#### 2. Outcomes Document
- **File**: `learning-outcomes.md`
- **Purpose**: Global success criteria for the entire course
- **Key Elements**:
  - Learning outcome statements
  - Assessment criteria
  - Skill domains
  - Application areas

#### 3. Structure Document
- **File**: `weekly-breakdown.md`
- **Purpose**: Meta-structure mapping weeks to modules
- **Key Elements**:
  - Week-to-module mapping
  - Instructor notes
  - Learning progression
  - Assessment schedule

#### 4. Requirements Document
- **File**: `hardware-requirements.md`
- **Purpose**: Authoritative reference for Physical AI infrastructure
- **Key Elements**:
  - Hardware specifications
  - Tier definitions
  - Cloud vs local constraints
  - Cost considerations
  - Safety infrastructure

#### 5. Architecture Document
- **File**: `lab-architecture.md`
- **Purpose**: Industry-grade system design explanation
- **Key Elements**:
  - Architecture patterns
  - Deployment models
  - System integration
  - Latency considerations
  - Security architecture

## Navigation Model

### Sidebar Structure
```js
{
  type: 'category',
  label: 'Course Foundations',
  items: [
    'course/why-physical-ai-matters',
    'course/learning-outcomes',
    'course/weekly-breakdown',
    'course/hardware-requirements',
    'course/lab-architecture'
  ],
}
```

### Cross-Reference Model
- Course foundations referenced from Module 4
- Relative paths: `../../course/document-name.md`
- Contextual linking with descriptive text

## Content Validation Rules

### Required Elements
- YAML frontmatter with sidebar_label and title
- H1 heading matching title
- Proper markdown formatting
- Clear section organization

### Quality Standards
- Technical and industry-oriented language
- Concise but comprehensive content
- Consistent formatting with existing materials
- Proper cross-references where applicable

## State Transitions

### Document Lifecycle
1. **Draft**: Initial content creation
2. **Review**: Content validation and formatting check
3. **Integrated**: Added to navigation and cross-referenced
4. **Published**: Available in course delivery