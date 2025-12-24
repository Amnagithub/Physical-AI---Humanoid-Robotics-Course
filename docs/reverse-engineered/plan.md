# Physical AI & Humanoid Robotics Course RAG Chatbot Implementation Plan

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-12-23

## Architecture Overview

**Architectural Style**: Microservice Architecture with layered design

**Reasoning**: The system follows a layered architecture pattern with clear separation of concerns to handle RAG-based question answering for educational content. The design separates concerns into presentation (React/Docusaurus), API (FastAPI), business logic (services), and data layers (PostgreSQL and Qdrant).

**Diagram** (ASCII):
```
[Student] -> [Docusaurus Frontend] -> [React Chat Component]
     |                                      |
     |<- Floating Chat Button with API calls-|
     |                                      |
     v                                      v
[API Gateway] -> [FastAPI Backend] -> [Session Management]
     |              |                        |
     |              |<- RAG Service <---------|
     |              |    |                   |
     |              |-> Embedding Service    |
     |              |    |                   |
     |              |-> Qdrant Service       |
     |              |    |                   |
     v              v    v                   v
[Client] <- [Response] [Vector DB] <- [Textbook Content]
                    |
                    v
              [PostgreSQL DB]
               [Sessions, Questions, Answers]
```

## Layer Structure

### Layer 1: [Presentation/API Layer]
- **Responsibility**: Handle HTTP requests, input validation, response formatting, and frontend integration
- **Components**:
  - [api/]: Request handlers (sessions.py, questions.py)
  - [middleware/]: Input validation, error handling
  - [frontend/src/components/]: React chat component (FloatingChatButton.jsx)
- **Dependencies**: → Service Layer
- **Technology**: FastAPI, React, Docusaurus

### Layer 2: [Business Logic/Service Layer]
- **Responsibility**: Core business rules for RAG processing, session management, and orchestration
- **Components**:
  - [services/]: RAGService, EmbeddingService, QdrantService, OpenAIService
  - [core business logic]: Question processing, response generation, source attribution
- **Dependencies**: → Data Layer, → External Services (OpenAI/Cohere)
- **Technology**: Python async services

### Layer 3: [Data/Persistence Layer]
- **Responsibility**: Data access, persistence, and retrieval
- **Components**:
  - [repositories/]: Database access objects (session, question, answer repos)
  - [models/]: SQLAlchemy ORM models
  - [database.py]: Database connection and session management
- **Dependencies**: → PostgreSQL Database, → Qdrant Vector Database
- **Technology**: SQLAlchemy, PostgreSQL, Qdrant

## Design Patterns Applied

### Pattern 1: [Repository Pattern]
- **Location**: [repositories/ directory]
- **Purpose**: Abstract data access from business logic
- **Implementation**: BaseRepository class with CRUD operations, specialized repositories for each entity

### Pattern 2: [Service Layer Pattern]
- **Location**: [services/ directory]
- **Purpose**: Encapsulate business logic and coordinate multiple data sources
- **Implementation**: RAGService orchestrates embedding, vector search, and LLM response generation

### Pattern 3: [Dependency Injection]
- **Location**: [config.py, service initialization]
- **Purpose**: Manage service dependencies and configuration
- **Implementation**: Settings class with environment-based configuration, service initialization in API routes

### Pattern 4: [API Resource Modeling]
- **Location**: [schemas/ directory]
- **Purpose**: Define clear contracts between API layers
- **Implementation**: Pydantic models for request/response validation

## Data Flow

### Question Processing Flow (Synchronous)
1. **API Layer** receives HTTP POST request to `/api/v1/sessions/{session_id}/questions`
2. **Validation Middleware** validates input schema (QuestionCreate)
3. **Authentication**: No auth required (anonymous sessions)
4. **Controller** (questions.py) routes to submit_question function
5. **Service Layer** (RAGService) executes business logic:
   - Creates question record in database
   - Generates embedding for query text
   - Searches Qdrant vector database for relevant content
   - Formats context and sends to LLM
   - Creates answer record with sources and confidence
6. **Repository Layer** persists question and answer to PostgreSQL
7. **Controller** formats response with AnswerResponse schema
8. **API Layer** returns JSON response to frontend

### Session Management Flow
1. **API Layer** receives session creation request
2. **Service Layer** creates session record with mode ('full_book' or 'selected_text')
3. **Repository Layer** persists session to database
4. **Response** includes session ID for subsequent requests

## Technology Stack

### Language & Runtime
- **Primary**: Python 3.8+
- **Rationale**: Rich ecosystem for AI/ML, async support, excellent for API development

### Web Framework
- **Choice**: FastAPI 0.104.1
- **Rationale**: Automatic API documentation, async support, Pydantic integration, excellent performance

### Database
- **Choice**: PostgreSQL (Neon) for structured data
- **Rationale**: ACID compliance, JSON support, connection pooling, mature ecosystem

### Vector Database
- **Choice**: Qdrant Cloud for embeddings
- **Rationale**: Purpose-built for similarity search, managed service, high performance

### Frontend Framework
- **Choice**: Docusaurus v3.9.2 with React
- **Rationale**: Static site generation for documentation, React component integration, excellent for educational content

### LLM Integration
- **Choice**: Multiple providers (OpenAI, Cohere, OpenRouter)
- **Rationale**: Provider flexibility, fallback options, competitive pricing

### Testing
- **Choice**: Built-in async testing, manual verification scripts
- **Rationale**: FastAPI testing utilities, asyncio support

### Deployment
- **Choice**: Static site hosting (GitHub Pages) + API backend
- **Rationale**: Separation of static content and dynamic API, cost-effective scaling

## Module Breakdown

### Module: [API Layer]
- **Purpose**: Handle HTTP requests, route to appropriate services
- **Key Classes**: FastAPI app, APIRouter instances
- **Dependencies**: Services, Database, Schemas
- **Complexity**: Medium

### Module: [RAG Service]
- **Purpose**: Core business logic for retrieval-augmented generation
- **Key Classes**: RAGService, EmbeddingService, QdrantService
- **Dependencies**: Vector DB, LLM providers, Content models
- **Complexity**: High

### Module: [Data Layer]
- **Purpose**: Data persistence and retrieval
- **Key Classes**: SQLAlchemy models, Repository classes
- **Dependencies**: PostgreSQL, Database connection
- **Complexity**: Medium

### Module: [Frontend Component]
- **Purpose**: Chat interface integrated into Docusaurus site
- **Key Classes**: FloatingChatButton React component
- **Dependencies**: Backend API, React ecosystem
- **Complexity**: Low to Medium

## Regeneration Strategy

### Option 1: Specification-First Rebuild
1. Start with spec.md (intent and requirements)
2. Apply extracted skills (error handling, API patterns)
3. Implement with modern best practices (fill gaps)
4. Test-driven development using acceptance criteria

**Timeline**: 6-8 weeks for full rebuild

### Option 2: Incremental Refactoring
1. **Strangler Pattern**: New implementation shadows old
2. **Feature Flags**: Gradual traffic shift
3. **Parallel Run**: Validate equivalence
4. **Cutover**: Complete migration

**Timeline**: 8-12 weeks with gradual migration

## Improvement Opportunities

### Technical Improvements
- [ ] **Fix Missing Logger** in questions.py
  - **Rationale**: Critical error handling bug
  - **Effort**: Low

- [ ] **Implement Content Ingestion Pipeline**
  - **Addresses Gap**: No mechanism to populate textbook content
  - **Effort**: Medium

- [ ] **Add Comprehensive Testing**
  - **Rationale**: Currently relies on manual test scripts
  - **Effort**: Medium

### Architectural Improvements
- [ ] **Add Circuit Breaker Pattern** for external API calls
  - **Enables**: More resilient error handling when LLM providers fail
  - **Effort**: Medium

- [ ] **Implement Caching Layer** for frequent questions
  - **Separates**: Reduces API costs and improves response time
  - **Effort**: Medium

### Operational Improvements
- [ ] **CI/CD Pipeline**: Automated testing, deployment
- [ ] **Infrastructure as Code**: Terraform, Pulumi
- [ ] **Monitoring Dashboards**: Grafana, DataDog
- [ ] **Rate Limiting**: Per-user/session request limits