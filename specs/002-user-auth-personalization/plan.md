# Implementation Plan: User Authentication & Personalized Learning Layer

**Branch**: `002-user-auth-personalization` | **Date**: 2025-12-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-user-auth-personalization/spec.md`

---

## Summary

Implement a comprehensive user authentication system using **Better Auth** (TypeScript) integrated with the existing **Docusaurus** frontend and **FastAPI** backend. The system will:

1. Enable email/password signup and signin with custom user profile fields (software/hardware background levels)
2. Provide per-chapter content personalization via LLM (Cohere) based on user profiles
3. Support Urdu translation of chapter content for authenticated users
4. Preserve original content integrity with reversible transformations

**Technical Approach**: Better Auth as a standalone Node.js authentication server, React auth client in Docusaurus, extended FastAPI backend for personalization/translation services, PostgreSQL (Neon) for all data storage.

---

## Technical Context

**Language/Version**: TypeScript 5.x (auth server), Python 3.11 (FastAPI), React 18 (frontend)
**Primary Dependencies**: Better Auth 1.x, Hono, FastAPI, Cohere SDK, Docusaurus 3.9.2
**Storage**: PostgreSQL (Neon Serverless) - shared database for auth and content
**Testing**: Vitest (TypeScript), pytest (Python), React Testing Library
**Target Platform**: Vercel (frontend + serverless), potential Railway/Render for auth server
**Project Type**: Web application (frontend + multiple backend services)
**Performance Goals**: Auth operations < 500ms, Personalization < 10s (with caching < 200ms)
**Constraints**: Zero modification of original markdown files, session-based auth, Urdu translation only
**Scale/Scope**: ~100 concurrent users, ~50 chapter documents, 7-day content cache

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| **I. Accuracy and Authority** | PASS | User profiles validated via schemas; LLM prompts ensure factual adaptation only |
| **II. Clarity and Accessibility** | PASS | Personalization explicitly adapts content to user background levels |
| **III. Reproducibility and Traceability** | PASS | Cache stores content with hashes; original content always preserved |
| **IV. Seamless Integration** | PASS | Auth integrated via theme swizzling; per-chapter controls injected naturally |
| **V. Functional Code and Validation** | PASS | API contracts defined with OpenAPI; Pydantic validation on all endpoints |
| **VI. Deployability and Reliability** | PASS | Existing Vercel deployment extended; cache prevents LLM overload |

**Additional Constraints Check**:
- [x] Docusaurus deployment maintained
- [x] Chatbot integration preserved (extends existing FastAPI)
- [x] Step-by-step setup documented in quickstart.md
- [x] Logical structure with cross-references

**GATE RESULT**: ✅ PASSED - No violations detected

---

## Project Structure

### Documentation (this feature)

```text
specs/002-user-auth-personalization/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 research findings
├── data-model.md        # Entity definitions and schemas
├── quickstart.md        # Developer setup guide
├── contracts/
│   ├── auth-api.yaml    # Better Auth endpoints (OpenAPI)
│   └── content-api.yaml # Personalization/translation endpoints (OpenAPI)
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
# Auth Server (NEW)
auth-server/
├── src/
│   ├── auth.ts          # Better Auth configuration
│   ├── index.ts         # Hono server entry point
│   └── types.ts         # TypeScript types
├── package.json
├── tsconfig.json
└── .env

# Frontend (EXISTING - extended)
frontend/
├── src/
│   ├── components/
│   │   ├── FloatingChatButton.jsx  # Existing
│   │   ├── AuthModal.jsx           # NEW: Login/Signup modal
│   │   ├── UserProfileButton.jsx   # NEW: Navbar user button
│   │   └── ChapterPersonalization.jsx # NEW: Per-chapter controls
│   ├── context/
│   │   └── AuthContext.jsx         # NEW: Auth state provider
│   ├── lib/
│   │   └── auth-client.ts          # NEW: Better Auth client
│   ├── theme/
│   │   ├── Root.js                 # MODIFIED: Add AuthProvider
│   │   └── DocItem/
│   │       └── Content/
│   │           └── index.jsx       # NEW: Swizzled for personalization
│   └── css/
│       └── auth.css                # NEW: Auth component styles
├── docusaurus.config.js            # MODIFIED: Add auth plugin
└── package.json                    # MODIFIED: Add dependencies

# Backend (EXISTING - extended)
backend/
├── src/
│   ├── api/
│   │   ├── sessions.py             # Existing
│   │   ├── questions.py            # Existing
│   │   └── personalization.py      # NEW: Personalization endpoints
│   ├── services/
│   │   ├── rag_service.py          # Existing
│   │   └── personalization_service.py # NEW: LLM personalization
│   ├── schemas/
│   │   └── personalization.py      # NEW: Pydantic models
│   ├── models/
│   │   └── personalized_content.py # NEW: SQLAlchemy model
│   └── repositories/
│       └── personalization_repository.py # NEW: DB operations
└── requirements.txt                # MODIFIED: Add new dependencies
```

**Structure Decision**: Web application pattern with separate auth server and extended backend. Auth server handles all authentication concerns; FastAPI handles content personalization. Frontend orchestrates both services.

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SYSTEM ARCHITECTURE                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        FRONTEND (Docusaurus)                         │   │
│  │  ┌───────────────┐  ┌───────────────┐  ┌─────────────────────────┐  │   │
│  │  │  AuthContext  │  │  AuthModal    │  │  ChapterPersonalization │  │   │
│  │  │  (Provider)   │  │  (Login/Sign) │  │  (Per-chapter controls) │  │   │
│  │  └───────┬───────┘  └───────┬───────┘  └───────────┬─────────────┘  │   │
│  │          │                  │                      │                 │   │
│  │          ▼                  ▼                      ▼                 │   │
│  │  ┌─────────────────────────────────────────────────────────────┐    │   │
│  │  │              Better Auth React Client                        │    │   │
│  │  │         useSession() | signIn() | signUp() | signOut()       │    │   │
│  │  └──────────────────────────────┬──────────────────────────────┘    │   │
│  └─────────────────────────────────┼───────────────────────────────────┘   │
│                                    │                                        │
│                    ┌───────────────┴────────────────┐                      │
│                    ▼                                ▼                      │
│  ┌─────────────────────────────┐    ┌─────────────────────────────────┐   │
│  │    AUTH SERVER (Hono)       │    │     FASTAPI BACKEND             │   │
│  │    /api/auth/*              │    │     /api/v1/*                   │   │
│  ├─────────────────────────────┤    ├─────────────────────────────────┤   │
│  │  • POST /sign-up/email      │    │  • POST /content/personalize    │   │
│  │  • POST /sign-in/email      │    │  • POST /content/translate      │   │
│  │  • POST /sign-out           │    │  • GET  /content/cache/{path}   │   │
│  │  • GET  /session            │    │  • Existing RAG endpoints       │   │
│  │  • POST /update-user        │    │                                 │   │
│  └──────────────┬──────────────┘    └────────────────┬────────────────┘   │
│                 │                                     │                    │
│                 │                                     ▼                    │
│                 │                    ┌─────────────────────────────────┐   │
│                 │                    │         COHERE LLM              │   │
│                 │                    │  • Personalization prompts      │   │
│                 │                    │  • Translation prompts          │   │
│                 │                    └─────────────────────────────────┘   │
│                 │                                                          │
│                 ▼                                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    POSTGRESQL (Neon Serverless)                      │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────────────────┐ │   │
│  │  │   user     │  │  session   │  │   personalized_content         │ │   │
│  │  │ +profile   │  │            │  │   (cache table)                │ │   │
│  │  └────────────┘  └────────────┘  └────────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Implementation Phases

### Phase 1: Authentication Infrastructure (P1 - Critical Path)

**Objective**: Establish Better Auth server and frontend auth integration

| Component | Deliverable | Acceptance Criteria |
|-----------|-------------|---------------------|
| Auth Server | `auth-server/` directory with Hono + Better Auth | Server starts, `/api/auth/session` returns empty |
| Database Schema | Better Auth tables + custom fields | `user` table has `softwareBackground`, `hardwareBackground` columns |
| Auth Client | `frontend/src/lib/auth-client.ts` | `useSession()` hook works in React components |
| AuthContext | `frontend/src/context/AuthContext.jsx` | Context provides user state to entire app |

**Dependencies**: PostgreSQL (Neon) - already available
**Risk**: CORS configuration between Docusaurus and auth server

### Phase 2: User Interface Components (P1)

**Objective**: Build signup, signin, and profile UI

| Component | Deliverable | Acceptance Criteria |
|-----------|-------------|---------------------|
| AuthModal | Signup/Signin modal with background selectors | User can register with email + profile |
| UserProfileButton | Navbar user button with dropdown | Shows user name when logged in |
| Profile Page | User profile update form | User can change background levels |
| Root.js Update | Integrate AuthProvider | Auth state available throughout app |

**Dependencies**: Phase 1 complete
**Risk**: Docusaurus theme swizzling compatibility

### Phase 3: Content Personalization Backend (P2)

**Objective**: Implement personalization and translation services

| Component | Deliverable | Acceptance Criteria |
|-----------|-------------|---------------------|
| Personalization API | `POST /api/v1/content/personalize` | Returns adapted content based on profile |
| Translation API | `POST /api/v1/content/translate` | Returns Urdu translation |
| Cache Layer | `personalized_content` table + repository | Cached content served < 200ms |
| LLM Prompts | Personalization and translation prompts | Content adapts appropriately to levels |

**Dependencies**: Phase 1 complete (for user authentication)
**Risk**: LLM response time > 10s SLA

### Phase 4: Chapter UI Integration (P2)

**Objective**: Add per-chapter personalization controls

| Component | Deliverable | Acceptance Criteria |
|-----------|-------------|---------------------|
| ChapterPersonalization | Per-chapter control component | "Personalize" and "Translate" buttons appear |
| DocItem Swizzle | Inject controls into chapter pages | Controls visible on all `/docs/*` pages |
| Content State | Client-side original/personalized toggle | User can revert to original instantly |
| Auth Gate | Disable controls for unauthenticated | Non-logged users see "Sign in to personalize" |

**Dependencies**: Phase 2 and Phase 3 complete
**Risk**: DOM manipulation conflicts with Docusaurus hydration

### Phase 5: Testing & Polish (P3)

**Objective**: Comprehensive testing and production hardening

| Component | Deliverable | Acceptance Criteria |
|-----------|-------------|---------------------|
| Auth Tests | Unit + integration tests for auth flows | 90%+ coverage on auth server |
| API Tests | pytest tests for personalization endpoints | All success/error paths tested |
| E2E Tests | Full signup → personalize → translate flow | Automated browser test passes |
| Rate Limiting | Auth + personalization rate limits | 5 signup/min, 10 personalize/min |
| Error Handling | User-friendly error messages | LLM failures show graceful fallback |

**Dependencies**: Phase 4 complete
**Risk**: Test environment database isolation

---

## Risk Analysis

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Better Auth + Docusaurus integration issues | High | Medium | Prototype auth server separately; test CORS early |
| LLM personalization exceeds 10s SLA | Medium | Medium | Aggressive caching; streaming responses |
| Docusaurus hydration conflicts | Medium | Low | Use portal rendering for modals; test SSR |
| Database schema conflicts with existing tables | High | Low | Use separate schema/prefix; test migrations |
| Translation quality insufficient | Medium | Medium | Iterate on prompts; consider fallback translation API |

---

## Success Metrics Mapping

| Spec Metric | Implementation | Measurement |
|-------------|----------------|-------------|
| SC-001: Signup < 2 min | Simple form, minimal fields | Time-to-complete tracking |
| SC-002: Signin < 30s | Email/password only | API response time |
| SC-003: Personalization < 10s | LLM + caching | API response time (p95) |
| SC-004: Translation < 10s | LLM + caching | API response time (p95) |
| SC-005: 100% reversible | Original content preserved | State management verification |
| SC-006: 99% auth uptime | Better Auth reliability | Uptime monitoring |
| SC-007: Zero file overwrites | Generated content in DB only | File integrity checks |
| SC-008: Disabled for unauth | Auth gate on all controls | UI state verification |

---

## Complexity Tracking

No constitution violations requiring justification. Architecture follows minimal complexity:
- Single auth server (not distributed)
- Extends existing FastAPI (no new service)
- Simple caching strategy (time-based expiry)
- No new infrastructure beyond auth server

---

## Generated Artifacts

| Artifact | Path | Description |
|----------|------|-------------|
| Research | `specs/002-user-auth-personalization/research.md` | Technology research and decisions |
| Data Model | `specs/002-user-auth-personalization/data-model.md` | Entity definitions and schemas |
| Auth API Contract | `specs/002-user-auth-personalization/contracts/auth-api.yaml` | OpenAPI spec for auth endpoints |
| Content API Contract | `specs/002-user-auth-personalization/contracts/content-api.yaml` | OpenAPI spec for personalization |
| Quickstart | `specs/002-user-auth-personalization/quickstart.md` | Developer setup guide |

---

## Next Steps

1. Run `/sp.tasks` to generate detailed implementation tasks
2. Begin Phase 1 implementation (auth server setup)
3. Set up development environment per quickstart.md

---

## ADR Suggestions

Based on architectural decisions made during planning:

> **Architectural decision detected**: Selection of Better Auth over NextAuth/custom JWT for authentication framework.
> Document reasoning and tradeoffs? Run `/sp.adr better-auth-selection`

> **Architectural decision detected**: Separate Node.js auth server vs. embedding auth in FastAPI.
> Document reasoning and tradeoffs? Run `/sp.adr auth-server-architecture`

> **Architectural decision detected**: Client-side content state management for personalization toggle.
> Document reasoning and tradeoffs? Run `/sp.adr content-state-management`
