---
id: 0001
title: Fix Docusaurus BaseURL Configuration
stage: general
date: 2025-12-21
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: amna
command: /sp.phr
labels: ["docusaurus", "configuration", "github-pages", "baseUrl", "deployment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/c/Users/TLS/Documents/GitHub/Physical-AI---Humanoid-Robotics-Course/frontend/docusaurus.config.js
tests:
 - none
---

## Prompt

--title "Fix-Docusaurus-BaseURL-Configuration" --stage general

## Response snapshot

Fixed Docusaurus site loading issue by updating configuration values in docusaurus.config.js: changed URL from placeholder to 'https://Amnagithub.github.io', updated organizationName to 'Amnagithub', fixed editUrl to point to correct repository, and corrected GitHub links in navbar and footer.

## Outcome

- ✅ Impact: Fixed Docusaurus site loading issue by correcting baseUrl and related GitHub configuration values that were using placeholders
- 🧪 Tests: none
- 📁 Files: /mnt/c/Users/TLS/Documents/GitHub/Physical-AI---Humanoid-Robotics-Course/frontend/docusaurus.config.js
- 🔁 Next prompts: none
- 🧠 Reflection: Proper GitHub Pages configuration requires updating all placeholder values in docusaurus.config.js, not just the baseUrl

## Evaluation notes (flywheel)

- Failure modes observed: Script .specify/scripts/bash/create-phr.sh not found, had to create PHR manually from template
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure PHR creation script is available in expected location