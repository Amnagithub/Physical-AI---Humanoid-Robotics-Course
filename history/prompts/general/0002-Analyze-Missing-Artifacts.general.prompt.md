---
id: 0002
title: Analyze Missing Artifacts
stage: general
date: 2025-12-21
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: amna
command: /sp.phr
labels: ["analysis", "missing-files", "workflow", "sdd"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - none
tests:
 - none
---

## Prompt

--title "Analyze-Missing-Artifacts" --stage general

## Response snapshot

Explained that /sp.analyze requires existing spec.md, plan.md, and tasks.md files which don't exist yet, and provided the correct workflow sequence to create them first.

## Outcome

- ✅ Impact: Clarified prerequisite workflow for /sp.analyze command and explained missing file dependencies
- 🧪 Tests: none
- 📁 Files: none
- 🔁 Next prompts: /sp.specify to create feature specification
- 🧠 Reflection: /sp.analyze is a validation tool that requires all three artifacts to exist first

## Evaluation notes (flywheel)

- Failure modes observed: Script .specify/scripts/bash/create-phr.sh not found, had to create PHR manually from template
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure PHR creation script is available in expected location