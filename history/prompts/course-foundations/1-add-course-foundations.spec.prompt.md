---
id: 1
title: add-course-foundations
stage: spec
date: '2025-12-20'
model: Claude Sonnet 4.5
feature: course-foundations
branch: 005-course-foundations
user: amna
command: /sp.specify
labels:
  - documentation
  - course-foundations
  - physical-ai
  - docusaurus
links:
  spec: specs/005-course-foundations/spec.md
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
  - docs/course/why-physical-ai-matters.md
  - docs/course/learning-outcomes.md
  - docs/course/weekly-breakdown.md
  - docs/course/hardware-requirements.md
  - docs/course/lab-architecture.md
  - frontend/sidebars.js
  - frontend/docs/modules/module-4/index.md
tests:
  - none
---

# PHR: add-course-foundations

## Prompt

```
/sp.specify

I want to extend my Docusaurus-based Physical AI & Humanoid Robotics course
by adding foundational course-level documentation before Module 1.

The following standalone documents must be added under docs/course/:

1) "Why Physical AI Matters"
   - File: docs/course/why-physical-ai-matters.md
   - Purpose: Course philosophy and motivation
   - Must explain the shift from digital AI to embodied intelligence
   - Must be linked before Module 1 as required reading

2) "Learning Outcomes"
   - File: docs/course/learning-outcomes.md
   - Purpose: Global success criteria for the entire course
   - Keep learning outcomes exactly as provided
   - Outcomes apply across all modules

3) "Weekly Breakdown"
   - File: docs/course/weekly-breakdown.md
   - Purpose: Meta-structure mapping weeks to modules
   - Not a teaching module
   - Include this mapping:
     - Weeks 1–2 → Module 1
     - Weeks 3–5 → Module 1 (Advanced)
     - Weeks 6–7 → Module 2
     - Weeks 8–10 → Module 3
     - Weeks 11–13 → Module 4

4) "Hardware Requirements"
   - File: docs/course/hardware-requirements.md
   - Purpose: Authoritative reference for Physical AI infrastructure
   - Must include:
     - Digital Twin Workstation
     - Jetson Edge Kits
     - Robot lab tiers
     - Cloud vs local constraints
   - Must be explicitly referenced in Module 3 and Module 4

5) "Lab Architecture & Deployment Models"
   - File: docs/course/lab-architecture.md
   - Purpose: Industry-grade system design explanation
   - Must explain:
     - Digital Twin workstation
     - Edge AI (Jetson)
     - Robot lab options
     - Cloud ("Ether Lab") alternative
     - Latency trap and sim-to-real workflow

Constraints:
- Technology: Docusaurus documentation site
- All files must be written in Markdown (.md)
- Content must be concise, technical, and industry-oriented
- These documents are course-level, not module-level

Module 4 dependency note:
- Module 4 (VLA + GPT) must reference:
  - hardware-requirements.md
  - lab-architecture.md
- Module 4 intro must clearly state this is where Physical AI becomes interactive
```

## Response

Successfully created foundational course-level documentation for the Physical AI & Humanoid Robotics course:

1. Created "Why Physical AI Matters" document explaining the shift from digital AI to embodied intelligence
2. Created "Learning Outcomes" document with global success criteria applying across all modules
3. Created "Weekly Breakdown" document mapping weeks to modules (Weeks 1-2 → Module 1, Weeks 3-5 → Module 1 Advanced, Weeks 6-7 → Module 2, Weeks 8-10 → Module 3, Weeks 11-13 → Module 4)
4. Created "Hardware Requirements" document with specifications for Digital Twin Workstation, Jetson Edge Kits, Robot lab tiers, and cloud vs local constraints
5. Created "Lab Architecture & Deployment Models" document explaining Digital Twin workstation, Edge AI (Jetson), Robot lab options, Cloud ("Ether Lab") alternative, and latency trap considerations
6. Updated Module 4 index to reference hardware requirements and lab architecture documents
7. Updated Module 4 intro to clearly state this is where Physical AI becomes interactive
8. Updated sidebar navigation to include Course Foundations category before Modules

All documents are written in Markdown format, are concise and technical, and serve as course-level rather than module-level documentation. The Module 4 content now properly references the hardware requirements and lab architecture documents as required.

## Outcome

The Physical AI & Humanoid Robotics course now has comprehensive foundational documentation that provides philosophical grounding, learning outcomes, scheduling structure, hardware requirements, and lab architecture before students begin Module 1. This ensures students understand the importance of Physical AI, know what they'll achieve, understand the course structure, and have access to necessary infrastructure information before diving into technical content.

## Evaluation

All requirements have been met successfully. The course foundations provide essential context and information for students before they begin the technical modules, and proper integration has been established with Module 4 as specified.