# Implementation Plan: Cohere-based RAG Chatbot

**Feature**: 1-rag-chatbot
**Created**: 2025-12-24
**Status**: Draft
**Author**: Claude

## Technical Context

This implementation plan outlines the architecture and development approach for an embedded RAG chatbot in a published book. The system will use Cohere for LLM and embeddings, FastAPI backend, Qdrant Cloud for vector search, and Neon Serverless Postgres for metadata and session management.

The system must support two modes:
1. Full Book RAG (default) - answers using entire book content
2. Selected Text Only (strict) - answers using only user-selected text

### Architecture Overview

- **Frontend**: Docusaurus-based book interface with embedded chatbot UI
- **Backend**: FastAPI service handling RAG logic and Cohere integration
- **Vector Store**: Qdrant Cloud for book content indexing and retrieval
- **Database**: Neon Serverless Postgres for session management and metadata
- **AI Service**: Cohere API for natural language processing and response generation

### Technology Stack

- **Backend Framework**: FastAPI (Python)
- **AI Provider**: Cohere API (LLM and embeddings)
- **Vector Database**: Qdrant Cloud
- **SQL Database**: Neon Serverless Postgres
- **Frontend Framework**: Docusaurus (existing)
- **Deployment**: Qween CLI
- **Prohibited**: OpenAI APIs or SDKs (strictly disallowed)

## Constitution Check

### Compliance Assessment

- ✅ **Accuracy and Authority**: RAG system will ensure answers are sourced only from book content with zero hallucination tolerance
- ✅ **Clarity and Accessibility**: UI will be designed for intuitive interaction with clear refusal messaging
- ✅ **Reproducibility and Traceability**: All components will be documented with setup instructions
- ✅ **Seamless Integration**: Chatbot will be embedded within book interface
- ✅ **Functional Code and Validation**: All code will be tested and validated
- ✅ **Deployability and Reliability**: Deployment will be automated with high availability via Qween CLI

### Risk Mitigation

- **Hallucination Prevention**: Strict content filtering and source attribution with exact refusal message
- **Performance**: Caching strategies and optimized vector search to meet ≤ 2s latency
- **Scalability**: Serverless architecture with auto-scaling capabilities
- **Security**: Proper credential handling via environment variables only
- **Compliance**: Strict adherence to using Cohere (not OpenAI) as required

## Implementation Phases

### Phase 1: Infrastructure Setup
- Set up environment variables and configuration
- Configure Qdrant Cloud connection
- Configure Neon Postgres connection
- Set up Cohere API integration
- Create basic FastAPI structure

### Phase 2: Core RAG Functionality
- Implement book content ingestion and chunking
- Generate Cohere embeddings for book content
- Store embeddings in Qdrant with metadata
- Implement semantic retrieval (Top-K ≤ 5)
- Implement basic answer generation

### Phase 3: Advanced Features
- Implement "Selected Text Only" mode
- Implement session management
- Add retrieval confidence checking (≥ 0.85)
- Implement refusal logic for insufficient context
- Add response latency monitoring

### Phase 4: Frontend Integration
- Create embeddable React chat component
- Implement API communication
- Add UI for both Full Book and Selected Text modes
- Implement loading states and error handling

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Frontend      │    │   FastAPI        │    │   Cohere API    │
│   (React)       │◄──►│   (Backend)      │◄──►│   (LLM/Embed)   │
│                 │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                            │    │
                            ▼    ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   Qdrant        │    │   Neon Postgres │
                    │   (Vector DB)   │    │   (Relational)  │
                    │                 │    │                 │
                    └─────────────────┘    └─────────────────┘
```

## Data Model Design

### Entity Relationships

```
UserSession
├── id: UUID
├── created_at: DateTime
├── last_activity: DateTime
├── mode: Enum('full_book', 'selected_text')
└── metadata: JSON

Question
├── id: UUID
├── session_id: UUID (foreign key to UserSession)
├── content: Text
├── created_at: DateTime
├── selected_text: Text (nullable)
└── context_mode: Enum('full_book', 'selected_text')

Answer
├── id: UUID
├── question_id: UUID (foreign key to Question)
├── content: Text
├── sources: JSON (array of source references)
├── created_at: DateTime
└── confidence_score: Float

BookContent
├── id: UUID
├── section: String
├── content: Text
├── embedding: Vector
├── metadata: JSON (book_title, chapter, section, page_range)
└── created_at: DateTime
```

## API Contracts

### REST API Endpoints

#### Session Management
- `POST /api/v1/sessions` - Create new session
- `GET /api/v1/sessions/{session_id}` - Get session details
- `PUT /api/v1/sessions/{session_id}/mode` - Update session mode

#### Question/Answer Interface
- `POST /api/v1/sessions/{session_id}/questions` - Submit question and get answer
- `GET /api/v1/sessions/{session_id}/history` - Get conversation history

#### Content Management
- `GET /api/v1/content/search` - Search book content
- `POST /api/v1/content/index` - Index new content (admin only)

### OpenAPI Specification

```yaml
openapi: 3.0.1
info:
  title: Cohere-based RAG Chatbot API
  version: 1.0.0
  description: API for the Cohere-powered RAG Chatbot embedded in published books

paths:
  /api/v1/sessions:
    post:
      summary: Create a new chat session
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                mode:
                  type: string
                  enum: [full_book, selected_text]
                  default: full_book
      responses:
        '201':
          description: Session created successfully
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Session'

  /api/v1/sessions/{session_id}/questions:
    post:
      summary: Submit a question and get an answer
      parameters:
        - name: session_id
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [content]
              properties:
                content:
                  type: string
                selected_text:
                  type: string
      responses:
        '200':
          description: Answer generated successfully
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Answer'

components:
  schemas:
    Session:
      type: object
      properties:
        id:
          type: string
        created_at:
          type: string
          format: date-time
        mode:
          type: string
          enum: [full_book, selected_text]

    Question:
      type: object
      properties:
        id:
          type: string
        content:
          type: string
        selected_text:
          type: string

    Answer:
      type: object
      properties:
        id:
          type: string
        content:
          type: string
        sources:
          type: array
          items:
            type: string
        confidence_score:
          type: number
          minimum: 0
          maximum: 1
```

## Development Environment Setup

### Prerequisites
- Python 3.9+
- Node.js 16+
- Access to Cohere API
- Qdrant Cloud account
- Neon Postgres account

### Setup Process

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Set up backend environment**
   ```bash
   # Create virtual environment
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install backend dependencies
   pip install fastapi uvicorn cohere python-dotenv qdrant-client asyncpg sqlalchemy pydantic-settings
   ```

3. **Configure environment variables**
   ```bash
   # Create .env file
   touch .env

   # Add required environment variables:
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_CLUSTER_ID=your_qdrant_cluster_id
   NEON_DATABASE_URL=your_neon_database_url
   ```

4. **Set up frontend (Docusaurus)**
   ```bash
   cd frontend
   npm install
   ```

5. **Run the application**
   ```bash
   # Terminal 1: Start the backend
   cd backend
   uvicorn main:app --reload

   # Terminal 2: Start the frontend
   cd frontend
   npm start
   ```

### Key Development Tasks

1. Set up project structure and dependencies with Cohere integration
2. Implement basic FastAPI backend with health check
3. Create database models for session management
4. Implement Qdrant vector storage integration with Cohere embeddings
5. Build RAG functionality with semantic retrieval (Top-K ≤ 5)
6. Implement content chunking and ingestion pipeline
7. Integrate with Docusaurus frontend
8. Implement both required modes (Full Book and Selected Text Only)
9. Add refusal logic with exact message when context insufficient
10. Add performance monitoring for ≤ 2s latency requirement
11. Package for deployment via Qween CLI

## Quality Assurance

### Testing Strategy
- Unit tests for core RAG functionality
- Integration tests for API endpoints
- Performance tests for latency requirements
- End-to-end tests for user flows

### Validation Criteria
- All responses must be grounded in book content only
- Refusal message must match exactly: "The provided text does not contain sufficient information to answer this question."
- Retrieval relevance ≥ 0.85
- Response latency ≤ 2s
- Zero hallucination tolerance