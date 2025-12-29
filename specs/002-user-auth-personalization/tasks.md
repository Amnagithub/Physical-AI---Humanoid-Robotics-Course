# Tasks: User Authentication & Personalized Learning Layer

**Input**: Design documents from `/specs/002-user-auth-personalization/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested in spec - test tasks omitted. Add manually if TDD approach desired.

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US5)
- Include exact file paths in descriptions

## Path Conventions

- **Auth Server**: `auth-server/src/`
- **Frontend**: `frontend/src/`
- **Backend**: `backend/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization for auth server and dependency updates

- [x] T001 Create auth-server directory structure: auth-server/src/, auth-server/package.json, auth-server/tsconfig.json
- [x] T002 Initialize auth-server Node.js project with dependencies: better-auth, pg, hono, @hono/node-server, dotenv
- [x] T003 [P] Add TypeScript dev dependencies to auth-server: typescript, @types/node, tsx
- [x] T004 [P] Create auth-server/.env with DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL
- [x] T005 [P] Add better-auth React client dependency to frontend/package.json
- [x] T006 [P] Update backend/requirements.txt with new dependencies for personalization

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Create Better Auth configuration in auth-server/src/auth.ts with PostgreSQL adapter and custom user fields (softwareBackground, hardwareBackground, preferredLanguage)
- [x] T008 Create Hono server entry point in auth-server/src/index.ts with CORS and /api/auth/* route handler
- [x] T009 Create TypeScript types in auth-server/src/types.ts for User, Session, BackgroundLevel
- [ ] T010 Run `npx better-auth generate` to create database schema migrations
- [ ] T011 Run `npx better-auth migrate` to apply Better Auth tables to Neon PostgreSQL
- [x] T012 [P] Create Better Auth React client in frontend/src/lib/auth-client.ts with baseURL and inferAdditionalFields plugin
- [x] T013 [P] Create AuthContext provider in frontend/src/context/AuthContext.jsx with useSession, signIn, signUp, signOut
- [x] T014 Update frontend/src/theme/Root.js to wrap children with AuthProvider
- [x] T015 [P] Create personalized_content table migration in backend/src/migrations/ per data-model.md SQL schema
- [ ] T016 Apply personalized_content migration to Neon PostgreSQL database
- [x] T017 [P] Create Pydantic schemas in backend/src/schemas/personalization.py (BackgroundLevel, ContentType, PersonalizationRequest, TranslationRequest, ContentResponse)
- [x] T018 [P] Create SQLAlchemy model in backend/src/models/personalized_content.py for PersonalizedContent entity
- [x] T019 Create personalization repository in backend/src/repositories/personalization_repository.py with CRUD operations for cache

**Checkpoint**: Foundation ready - auth server running, frontend client configured, backend schema ready

---

## Phase 3: User Story 1 - New Learner Registration (Priority: P1) MVP

**Goal**: Allow new learners to create accounts with email/password and background profile

**Independent Test**: Complete signup flow end-to-end, verify user record created with profile data in database

### Implementation for User Story 1

- [x] T020 [P] [US1] Create SignupForm component in frontend/src/components/SignupForm.jsx with email, password, name, softwareBackground, hardwareBackground fields
- [x] T021 [P] [US1] Create BackgroundSelector component in frontend/src/components/BackgroundSelector.jsx for beginner/intermediate/advanced selection
- [x] T022 [US1] Create AuthModal component in frontend/src/components/AuthModal.jsx with signup/signin tabs
- [x] T023 [US1] Add signup form validation: email format, password min 8 chars, required name
- [x] T024 [US1] Implement signup submission in SignupForm using authClient.signUp.email() with custom fields
- [x] T025 [US1] Handle signup errors: duplicate email (409), validation errors (400), display user-friendly messages
- [x] T026 [US1] Implement post-signup redirect to /docs course content
- [x] T027 [P] [US1] Create auth.css styles in frontend/src/css/auth.css for modal and form components
- [x] T028 [US1] Add "Sign Up" button to navbar that opens AuthModal - in frontend/src/theme/Navbar/ or via swizzle

**Checkpoint**: User Story 1 complete - users can register with profile. Test by creating account and verifying database entry.

---

## Phase 4: User Story 2 - Returning Learner Sign In (Priority: P1)

**Goal**: Allow returning learners to sign in and restore their session

**Independent Test**: Sign in with valid credentials, verify session cookie set, user data available in context

### Implementation for User Story 2

- [x] T029 [P] [US2] Create SigninForm component in frontend/src/components/SigninForm.jsx with email, password, rememberMe fields
- [x] T030 [US2] Integrate SigninForm into AuthModal as signin tab
- [x] T031 [US2] Implement signin submission using authClient.signIn.email() with rememberMe option
- [x] T032 [US2] Handle signin errors: invalid credentials (401), display error message
- [x] T033 [US2] Implement post-signin redirect to /docs or previous page
- [x] T034 [US2] Create UserProfileButton component in frontend/src/components/UserProfileButton.jsx showing user name when logged in
- [x] T035 [US2] Add UserProfileButton to navbar with dropdown menu (Profile, Sign Out)
- [x] T036 [US2] Implement signOut functionality in UserProfileButton using authClient.signOut()
- [x] T037 [US2] Handle session persistence: verify user stays logged in after browser close (rememberMe=true)
- [x] T038 [US2] Update AuthModal to show signin by default for returning visitors

**Checkpoint**: User Story 2 complete - users can sign in/out. Test signin flow and session persistence.

---

## Phase 5: User Story 3 - Personalize Chapter Content (Priority: P2)

**Goal**: Allow authenticated users to personalize chapter content based on their background

**Independent Test**: Click "Personalize" on a chapter, verify content adapts to user profile, verify "Show Original" restores content

### Implementation for User Story 3

- [x] T039 [P] [US3] Create personalization service in backend/src/services/personalization_service.py with Cohere LLM integration
- [x] T040 [US3] Implement personalization prompt template in personalization_service.py that adapts content based on software/hardware background levels
- [x] T041 [US3] Implement cache-first logic in personalization_service: check cache, generate if miss, store result
- [x] T042 [US3] Create POST /api/v1/content/personalize endpoint in backend/src/api/personalization.py per content-api.yaml contract
- [x] T043 [US3] Add session validation to personalization endpoint - require authenticated user
- [x] T044 [US3] Register personalization router in backend/src/main.py
- [x] T045 [P] [US3] Create ChapterPersonalization component in frontend/src/components/ChapterPersonalization.jsx with "Personalize Content" button
- [x] T046 [US3] Implement content state management in ChapterPersonalization: original content, personalized content, activeView toggle
- [x] T047 [US3] Implement personalizeContent() function that calls POST /api/v1/content/personalize with chapter markdown and user profile
- [x] T048 [US3] Add "Show Original" button to ChapterPersonalization that restores original content
- [x] T049 [US3] Swizzle DocItem/Content to inject ChapterPersonalization component - run: npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap
- [x] T050 [US3] Create frontend/src/theme/DocItem/Content/index.jsx wrapper that adds ChapterPersonalization above content
- [x] T051 [US3] Implement auth gate: disable "Personalize" button for unauthenticated users with "Sign in to personalize" tooltip
- [x] T052 [US3] Add loading state and error handling for personalization API calls
- [x] T053 [US3] Handle LLM unavailable error (503) with user-friendly message and "Show Original" fallback

**Checkpoint**: User Story 3 complete - authenticated users can personalize content. Test with different background levels.

---

## Phase 6: User Story 4 - Translate Chapter Content to Urdu (Priority: P2)

**Goal**: Allow authenticated users to translate chapter content to Urdu

**Independent Test**: Click "Translate to Urdu" on a chapter, verify Urdu content displays, UI remains English, "Show Original" works

### Implementation for User Story 4

- [x] T054 [P] [US4] Create translation prompt template in backend/src/services/personalization_service.py for Urdu translation preserving code blocks
- [x] T055 [US4] Implement translateContent() method in personalization_service.py using Cohere LLM
- [x] T056 [US4] Create POST /api/v1/content/translate endpoint in backend/src/api/personalization.py per content-api.yaml contract
- [x] T057 [US4] Add "Translate to Urdu" button to ChapterPersonalization component in frontend/src/components/ChapterPersonalization.jsx
- [x] T058 [US4] Implement translateContent() function in ChapterPersonalization that calls POST /api/v1/content/translate
- [x] T059 [US4] Add translated content state management - track if content is English or Urdu
- [x] T060 [US4] Implement combined personalization + translation: allow user to personalize first, then translate
- [x] T061 [US4] Add "English" button that appears when viewing Urdu content to revert translation
- [x] T062 [US4] Implement auth gate for translate button matching personalize gate
- [x] T063 [US4] Add loading state for translation API calls (may take up to 10s)
- [x] T064 [US4] Handle translation service unavailable error with user-friendly message

**Checkpoint**: User Story 4 complete - authenticated users can translate content to Urdu. Test translation quality and combined personalize+translate.

---

## Phase 7: User Story 5 - Update User Profile (Priority: P3)

**Goal**: Allow authenticated users to update their background levels affecting future personalization

**Independent Test**: Update profile fields, verify changes persist, verify next personalization uses new levels

### Implementation for User Story 5

- [x] T065 [P] [US5] Create ProfilePage component in frontend/src/pages/profile.jsx with form for name, softwareBackground, hardwareBackground
- [x] T066 [US5] Add profile page route to Docusaurus - create frontend/src/pages/profile.js
- [x] T067 [US5] Implement profile form with current values pre-populated from AuthContext user
- [x] T068 [US5] Implement profile update submission using authClient.updateUser() with new field values
- [x] T069 [US5] Add success/error feedback for profile update
- [x] T070 [US5] Link to profile page from UserProfileButton dropdown menu
- [x] T071 [US5] Invalidate personalization cache when profile updated - call DELETE /api/v1/content/cache/{path} or clear all user cache
- [x] T072 [P] [US5] Create GET /api/v1/content/user-preferences endpoint in backend/src/api/personalization.py
- [x] T073 [P] [US5] Create DELETE /api/v1/content/cache/{chapter_path} endpoint for cache invalidation

**Checkpoint**: User Story 5 complete - users can update profile and get fresh personalization.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Production hardening, error handling, performance

- [x] T074 [P] Add rate limiting to auth-server: 5 signup/min, 10 signin/min per IP
- [x] T075 [P] Add rate limiting to personalization endpoints: 10 requests/min per user
- [ ] T076 [P] Implement cache cleanup job for expired personalized_content entries
- [ ] T077 Update frontend CORS configuration in docusaurus.config.js if needed
- [x] T078 [P] Add comprehensive error logging to auth-server/src/index.ts
- [ ] T079 [P] Add comprehensive error logging to backend/src/api/personalization.py
- [ ] T080 Create deployment configuration for auth-server (Vercel serverless or Railway)
- [ ] T081 Update frontend/.env.production with production auth server URL
- [ ] T082 Update backend/.env with production CORS origins
- [ ] T083 Run quickstart.md validation - verify all setup steps work
- [ ] T084 Verify SC-001: Signup completes in under 2 minutes
- [ ] T085 Verify SC-003/SC-004: Personalization/translation under 10 seconds (with caching)
- [ ] T086 Verify SC-007: Zero markdown file modifications after all operations

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational) ← BLOCKS ALL USER STORIES
    ↓
┌───────────────────────────────────────────────────────┐
│  Phase 3 (US1)  ←→  Phase 4 (US2)                     │  Can run in parallel
│       ↓                  ↓                            │  or sequentially
│  Phase 5 (US3)  ←→  Phase 6 (US4)  →  Phase 7 (US5)  │
└───────────────────────────────────────────────────────┘
    ↓
Phase 8 (Polish)
```

### User Story Dependencies

| Story | Depends On | Can Parallel With |
|-------|------------|-------------------|
| US1 (Signup) | Phase 2 | US2 (different forms) |
| US2 (Signin) | Phase 2 | US1 (different forms) |
| US3 (Personalize) | Phase 2 + US1/US2 auth UI | US4 (different backend methods) |
| US4 (Translate) | Phase 2 + US1/US2 auth UI | US3 (different backend methods) |
| US5 (Profile) | US1/US2 (need authenticated users) | None |

### Within Each User Story

- Models/schemas before services
- Services before API endpoints
- Backend endpoints before frontend integration
- Core implementation before error handling
- Complete story before moving to next priority

---

## Parallel Execution Examples

### Phase 2 Parallel Tasks
```
Parallel batch 1:
- T012: Create auth-client.ts
- T013: Create AuthContext.jsx
- T015: Create personalized_content migration
- T017: Create Pydantic schemas
- T018: Create SQLAlchemy model

Sequential after batch 1:
- T014: Update Root.js (depends on T013)
- T016: Apply migration (depends on T15)
- T019: Create repository (depends on T18)
```

### User Story 1 Parallel Tasks
```
Parallel:
- T020: Create SignupForm.jsx
- T021: Create BackgroundSelector.jsx
- T027: Create auth.css

Sequential:
- T022: Create AuthModal.jsx (uses T020, T021)
- T023-T028: Validation, submission, errors, redirect, navbar
```

### User Stories 3 & 4 Parallel Backend
```
Can develop simultaneously:
- Developer A: T039-T044 (personalization backend)
- Developer B: T054-T056 (translation backend)

Then integrate frontend together: T045-T064
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL)
3. Complete Phase 3: User Story 1 (Signup)
4. Complete Phase 4: User Story 2 (Signin)
5. **STOP and VALIDATE**: Users can sign up and sign in
6. Deploy MVP - authentication working

### Incremental Delivery

| Milestone | Stories | Value Delivered |
|-----------|---------|-----------------|
| MVP | US1 + US2 | Users can create accounts and sign in |
| +Personalization | US3 | Content adapts to user background |
| +Translation | US4 | Urdu accessibility for content |
| +Profile | US5 | Users can update preferences |

### Suggested First Session

Complete tasks T001-T019 (Setup + Foundational) to establish working infrastructure before any user story implementation.

---

## Task Summary

| Phase | Tasks | Parallel Opportunities |
|-------|-------|------------------------|
| Setup | T001-T006 (6) | T003, T004, T005, T006 |
| Foundational | T007-T019 (13) | T012-T013, T015-T018 |
| US1 - Signup | T020-T028 (9) | T020, T021, T027 |
| US2 - Signin | T029-T038 (10) | T029 |
| US3 - Personalize | T039-T053 (15) | T039, T045 |
| US4 - Translate | T054-T064 (11) | T054 |
| US5 - Profile | T065-T073 (9) | T065, T072, T073 |
| Polish | T074-T086 (13) | T074-T076, T078-T079 |

**Total Tasks**: 86
**MVP Tasks (US1+US2)**: 38 tasks (Setup + Foundational + US1 + US2)

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to user story for traceability
- All user stories independently testable after completion
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Original markdown files MUST NOT be modified (FR-015)
