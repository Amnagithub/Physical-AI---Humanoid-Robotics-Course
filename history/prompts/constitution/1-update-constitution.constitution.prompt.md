---
id: 1
title: Update Constitution
stage: constitution
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: TLS
command: /sp.constitution
labels: [constitution,documentation,setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: AI/Spec-Driven Book with Embedded RAG Chatbot

Core principles:
- Accuracy: All book content and chatbot answers must reflect verified, authoritative sources.
- Clarity: Writing must be accessible to technical and non-technical readers.
- Reproducibility: All processes, code, and data references must be traceable and executable.
- Integration: Book and chatbot must be seamlessly connected; chatbot answers must reference only user-selected text when applicable.

Key standards:
- Book authored using Docusaurus, deployed to GitHub Pages.
- RAG chatbot built using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud.
- Chatbot answers must be contextually accurate and limited to book content unless explicitly instructed otherwise.
- Code snippets, examples, and references must be validated and functional.
- Documentation clarity: step-by-step instructions for setup, deployment, and usage.

Constraints:
- Book structure: logical sections with headings, code examples, and references.
- Chatbot: responds to user-selected text queries accurately.
- Deployment: GitHub Pages live link, working chatbot embedded.
- Writing style: professional, concise, and consistent throughout.

Success criteria:
- Book fully deployed with Docusaurus on GitHub Pages.
- Chatbot accurately answers content-specific queries, including user-selected text.
- All code and references verified and reproducible.
- Integration between book content and chatbot verified and functional.
```

## Response snapshot

Created a comprehensive constitution for the AI/Spec-Driven Book with Embedded RAG Chatbot project based on the provided specifications. The constitution includes six core principles covering accuracy, clarity, reproducibility, seamless integration, functional code validation, and deployability. It also defines additional constraints around technology stack and deployment, development workflow requirements, and governance procedures.

## Outcome

- ✅ Impact: Created foundational constitution document that will guide all project decisions
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Project can now proceed with spec creation aligned to constitution
- 🧠 Reflection: Constitution captures all specified requirements and establishes clear governance framework

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin spec development aligned with constitution