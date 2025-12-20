---
id: 5
title: reverse-engineer-and-normalize
stage: refactor
date: '2025-12-20'
model: Claude Sonnet 4.5
feature: directory-normalization
branch: main
user: amna
command: sp.reverse-engineer
labels:
  - reverse-engineering
  - directory-structure
  - normalization
  - docusaurus
links:
  spec: docs/reverse-engineered-spec.md
  plan: docs/reverse-engineered-plan.md
  tasks: docs/reverse-engineered-tasks.md
  intelligence: docs/reverse-engineered-intelligence.md
  ticket: null
  adr: null
  pr: null
files:
  - docs/reverse-engineered-spec.md
  - docs/reverse-engineered-plan.md
  - docs/reverse-engineered-tasks.md
  - docs/reverse-engineered-intelligence.md
  - frontend/docusaurus.config.js
  - frontend/sidebars.js
  - docs/
  - specs/
  - history/
tests:
  - none
---

# PHR: reverse-engineer-and-normalize

## Prompt

```
I want to normalize and fix the directory structure of my Docusaurus-based
Physical AI & Humanoid Robotics course.

Some files and folders were placed in incorrect locations.all the specs should be in the spec folder and the history/prompt also be managed accordingly.
```

## Response

Successfully reverse-engineered and normalized the directory structure of the Docusaurus-based Physical AI & Humanoid Robotics course:

**Reverse Engineering Completed:**
- Analyzed current structure with scattered content
- Extracted specification from existing implementation
- Documented architecture and implementation patterns
- Created comprehensive task breakdown
- Extracted reusable intelligence from codebase

**Normalization Implemented:**
- Consolidated documentation content from frontend/docs/ to root docs/
- Consolidated specs from frontend/specs/ to root specs/
- Consolidated history files from frontend/history/ to root history/
- Updated Docusaurus configuration to reference correct paths
- Removed duplicate content from frontend directories
- Fixed broken links in footer navigation

**Final Structure:**
- docs/ (all documentation content including course/ and modules/)
- specs/ (all specifications organized by module number 001-005)
- history/ (all prompt history records organized by feature)

**Verification:**
- Docusaurus site builds successfully with normalized structure
- All navigation and links work correctly
- Content remains accessible and functional
- Configuration properly references new directory structure

## Outcome

The Physical AI & Humanoid Robotics course now has a normalized directory structure with all content in appropriate locations. The reverse engineering process extracted valuable specifications, plans, tasks, and reusable intelligence from the existing implementation. The normalization consolidates scattered files into proper locations while maintaining full functionality.

## Evaluation

All requirements have been met successfully. The directory structure is now normalized with:
- All documentation in root docs/ directory
- All specifications in root specs/ directory
- All history/prompt files in root history/ directory
- Proper Docusaurus configuration referencing new paths
- Successful build and functionality verification