---
id: 002
title: Better Auth Implementation Plan
stage: plan
date: 2025-12-26
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-user-auth-personalization
branch: 002-user-auth-personalization
user: TLS
command: /sp.plan
labels: ["better-auth", "docusaurus", "authentication", "personalization", "planning"]
links:
  spec: specs/002-user-auth-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-user-auth-personalization/plan.md
  - specs/002-user-auth-personalization/research.md
  - specs/002-user-auth-personalization/data-model.md
  - specs/002-user-auth-personalization/quickstart.md
  - specs/002-user-auth-personalization/contracts/auth-api.yaml
  - specs/002-user-auth-personalization/contracts/content-api.yaml
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan
Implement Better Auth–based signup and signin, collecting user software and hardware background to enable
authenticated personalization of chapter content in the Physical AI & Humanoid Robotics Course.
Allow logged-in users to personalize and translate chapter content into Urdu via per-chapter controls while
preserving original content integrity. Credentials provided:
- BETTER_AUTH_SECRET=e8o8oJh/rFV7IafAGRKzzlxELIOHzGf27U3H4FJNe5E=
- BETTER_AUTH_URL=https://physical-ai-humanoid-robotics-cours-ashen.vercel.app
Platform: Docusaurus

## Response snapshot

### Planning Execution Summary

Executed the `/sp.plan` workflow to create comprehensive implementation planning artifacts for the User Authentication & Personalized Learning Layer feature.

**Key Decisions Made:**

1. **Architecture**: Better Auth as standalone Node.js server (Hono) + extended FastAPI backend for personalization
2. **Database**: Extend PostgreSQL (Neon) with Better Auth tables + custom `personalized_content` cache table
3. **Frontend**: Docusaurus theme swizzling for auth integration + React Context for state management
4. **Personalization**: LLM-based (Cohere) content adaptation with 7-day caching strategy

**Artifacts Generated:**

| Artifact | Purpose |
|----------|---------|
| `plan.md` | Main implementation plan with 5 phases, architecture diagram, risk analysis |
| `research.md` | Technology research: Better Auth patterns, Docusaurus integration, LLM prompts |
| `data-model.md` | Entity definitions, SQL schema, TypeScript/Pydantic models |
| `auth-api.yaml` | OpenAPI contract for Better Auth endpoints |
| `content-api.yaml` | OpenAPI contract for personalization/translation endpoints |
| `quickstart.md` | Developer setup guide with verification steps |

**Implementation Phases:**

1. Phase 1: Authentication Infrastructure (P1) - Auth server + frontend client
2. Phase 2: User Interface Components (P1) - Signup/signin modals, profile
3. Phase 3: Content Personalization Backend (P2) - LLM personalization + translation
4. Phase 4: Chapter UI Integration (P2) - Per-chapter controls
5. Phase 5: Testing & Polish (P3) - Comprehensive testing, rate limiting

**Constitution Check**: All 6 principles PASSED

## Outcome

- ✅ Impact: Created comprehensive implementation plan enabling team to begin development with clear architecture and contracts
- 🧪 Tests: N/A (planning phase - tests defined in Phase 5)
- 📁 Files: 6 artifacts created in specs/002-user-auth-personalization/
- 🔁 Next prompts: Run `/sp.tasks` to generate detailed implementation tasks, then begin Phase 1
- 🧠 Reflection: Better Auth integration with Docusaurus requires careful CORS handling; caching strategy critical for LLM performance targets

## Evaluation notes (flywheel)

- Failure modes observed: None during planning; potential risks documented in plan.md
- Graders run and results (PASS/FAIL): Constitution Check PASSED all 6 principles
- Prompt variant (if applicable): Standard /sp.plan with provided credentials
- Next experiment (smallest change to try): Prototype auth server CORS configuration before full implementation
