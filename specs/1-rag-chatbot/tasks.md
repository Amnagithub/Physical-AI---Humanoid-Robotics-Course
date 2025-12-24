# Tasks: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Feature Overview

Build an embedded RAG chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant that answers questions strictly from the book's content. Support both full-book retrieval and user-selected text–only mode, ensuring accurate, hallucination-free responses inside the published book.

**Feature Branch**: `1-rag-chatbot`
**Created**: 2025-12-22
**Status**: Task List

## Implementation Strategy

This feature will be implemented in phases following the user story priorities from the specification. The approach will be to build an MVP with User Story 1 (core Q&A functionality) first, then incrementally add the additional features for User Stories 2 and 3. Each user story will be independently testable and deliverable.

## Dependencies

- User Story 1 (P1) must be completed before User Story 2 (P2) and User Story 3 (P3)
- User Story 2 (P2) and User Story 3 (P3) can be developed in parallel after User Story 1 is complete
- All foundational components must be in place before any user story implementation begins

## Parallel Execution Examples

- Database models can be developed in parallel with API endpoint implementation
- Frontend components can be developed in parallel with backend services
- Textbook content indexing can run in parallel with API development

---

## Phase 1: Setup Tasks

### Goal
Set up the project structure, dependencies, and basic configuration for the RAG chatbot system.

### Independent Test Criteria
- Development environment can be set up following documented steps
- Basic backend service starts without errors
- All required services (database, vector store) can be connected

### Tasks

- [x] T001 Create backend directory structure (src/, tests/, config/, docs/)
- [x] T002 Set up Python virtual environment and requirements.txt with FastAPI, OpenAI, Qdrant, asyncpg
- [x] T003 Configure environment variables for API keys and service URLs
- [x] T004 Set up basic FastAPI application with health check endpoint
- [x] T005 [P] Create .gitignore for Python/Node.js project
- [x] T006 [P] Set up basic project documentation files (README.md, CONTRIBUTING.md)

---

## Phase 2: Foundational Tasks

### Goal
Implement foundational components that are required for all user stories: database models, vector storage integration, and core services.

### Independent Test Criteria
- Database schema can be created and connected successfully
- Vector storage can be initialized and connected
- Textbook content can be processed and stored in vector database
- Core services are available for user story implementation

### Tasks

- [x] T007 Create database models for StudentSession, Question, Answer, TextbookContent
- [x] T008 Set up database connection and migration system using asyncpg
- [x] T009 [P] Create database tables for all entities with proper relationships
- [x] T010 [P] Implement database repository classes for each entity
- [x] T011 Set up Qdrant client connection and configuration
- [x] T012 Create TextbookContent schema in Qdrant with proper vector indexing
- [x] T013 [P] Implement textbook content ingestion pipeline
- [x] T014 [P] Create embedding generation service using OpenAI API
- [x] T015 Implement RAG service with content retrieval functionality
- [x] T016 [P] Create OpenAI integration service for response generation
- [x] T017 Set up configuration management for different environments

---

## Phase 3: User Story 1 - Ask Questions about Textbook Content (Priority: P1)

### Goal
Implement core functionality where students can ask questions about textbook content and receive accurate answers based solely on the textbook.

### Independent Test Criteria
- Student can ask a question about any topic in the textbook and receive an accurate answer that is directly sourced from the textbook content, with no external information or hallucinations
- When a student asks a question not covered in the textbook, the chatbot responds with "This information is not covered in the book."

### Tasks

- [x] T018 [US1] Create API endpoint POST /api/sessions to create new chat sessions (default full_book mode)
- [x] T019 [US1] Create API endpoint POST /api/sessions/{session_id}/questions to submit questions and get answers
- [x] T020 [US1] Implement RAG logic for full-book mode question answering
- [x] T021 [US1] Implement content retrieval from Qdrant based on question context
- [x] T022 [US1] Generate responses using OpenAI API with textbook context
- [x] T023 [US1] Implement source attribution in responses with proper citations
- [x] T024 [US1] Handle cases where content is not found in textbook (return "This information is not covered in the book.")
- [x] T025 [US1] Implement confidence scoring for responses
- [x] T026 [US1] Add proper error handling and validation for question input
- [x] T027 [US1] Create basic frontend UI component for chat interface
- [x] T028 [US1] Integrate backend API with frontend chat component
- [x] T029 [US1] Implement response formatting with source citations
- [x] T030 [US1] Test core functionality with sample textbook questions

---

## Phase 4: User Story 2 - Contextual Question Answering with Selected Text (Priority: P2)

### Goal
Implement functionality where students can select specific text from the textbook and ask questions specifically about that selected content, with responses restricted to the selected text only.

### Independent Test Criteria
- Student can select text, ask a question about it, and receive answers that are exclusively based on the selected text, not the broader book content

### Tasks

- [ ] T031 [US2] Update API endpoint POST /api/sessions/{session_id}/questions to accept selected_text parameter
- [ ] T032 [US2] Implement selected text only mode in RAG service
- [ ] T033 [US2] Create logic to restrict content retrieval to selected text only
- [ ] T034 [US2] Update response generation to use only selected text context
- [ ] T035 [US2] Handle cases where selected text is insufficient (return "The selected text does not contain enough information to answer this question.")
- [ ] T036 [US2] Add mode switching functionality in session management
- [ ] T037 [US2] Create PUT /api/sessions/{session_id}/mode endpoint to update session mode
- [ ] T038 [US2] Update frontend to support text selection and mode switching
- [ ] T039 [US2] Implement visual indicators for selected text mode
- [ ] T040 [US2] Test selected text functionality with various text selections

---

## Phase 5: User Story 3 - Session Management and History (Priority: P3)

### Goal
Implement session management functionality allowing students to maintain context during their learning session with conversation history tracking.

### Independent Test Criteria
- Student can have a multi-turn conversation with the chatbot, with proper session management and history tracking

### Tasks

- [ ] T041 [US3] Create GET /api/sessions/{session_id} endpoint to retrieve session details
- [ ] T042 [US3] Create GET /api/sessions/{session_id}/history endpoint to get conversation history
- [ ] T043 [US3] Implement session timeout and cleanup functionality
- [ ] T044 [US3] Add conversation history tracking in database
- [ ] T045 [US3] Implement context maintenance for multi-turn conversations
- [ ] T046 [US3] Add last_activity tracking for session management
- [ ] T047 [US3] Implement session metadata storage and retrieval
- [ ] T048 [US3] Update frontend to display conversation history
- [ ] T049 [US3] Add session persistence across browser sessions
- [ ] T050 [US3] Test multi-turn conversation functionality

---

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Implement quality-of-life features, error handling, performance optimizations, and security measures.

### Independent Test Criteria
- System performs well under expected load
- Error conditions are handled gracefully
- Security measures are in place
- Performance requirements are met

### Tasks

- [ ] T051 Add comprehensive error handling and logging throughout the system
- [ ] T052 Implement rate limiting for API endpoints
- [ ] T053 Add input validation and sanitization for all user inputs
- [ ] T054 Implement caching for frequently accessed content
- [ ] T055 Add performance monitoring and metrics collection
- [ ] T056 Create comprehensive API documentation with OpenAPI/Swagger
- [ ] T057 Implement security headers and authentication if needed
- [ ] T058 Add comprehensive tests (unit, integration, end-to-end)
- [ ] T059 Optimize vector search performance and response times
- [ ] T060 Conduct security review and penetration testing
- [ ] T061 Create deployment configuration for production
- [ ] T062 Perform load testing to validate performance requirements
- [ ] T063 Create user documentation and help guides
- [ ] T064 Conduct final integration testing
- [ ] T065 Deploy to production environment

---

## MVP Scope

The MVP will include Phase 1 (Setup), Phase 2 (Foundational), and Phase 3 (User Story 1) tasks, providing core functionality where students can ask questions about textbook content and receive accurate, source-attributed answers.