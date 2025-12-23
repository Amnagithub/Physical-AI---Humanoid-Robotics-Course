# Implementation Plan: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

**Feature**: 1-rag-chatbot
**Created**: 2025-12-22
**Status**: Draft
**Author**: Claude

## Technical Context

This implementation plan outlines the architecture and development approach for an embedded RAG chatbot in the "Physical AI & Humanoid Robotics" textbook. The system will use OpenAI Agents/ChatKit SDKs, FastAPI backend, Qdrant Cloud for vector search, and Neon Serverless Postgres for metadata and session management.

The system must support two modes:
1. Full Book RAG (default) - answers using entire textbook content
2. Selected Text Only (strict) - answers using only user-selected text

### Architecture Overview

- **Frontend**: Docusaurus-based textbook interface with embedded chatbot UI
- **Backend**: FastAPI service handling RAG logic and OpenAI integration
- **Vector Store**: Qdrant Cloud for textbook content indexing and retrieval
- **Database**: Neon Serverless Postgres for session management and metadata
- **AI Service**: OpenAI API for natural language processing and response generation

### Technology Stack

- **Frontend Framework**: Docusaurus (existing)
- **Backend Framework**: FastAPI
- **AI SDK**: OpenAI Agents/ChatKit SDKs
- **Vector Database**: Qdrant Cloud
- **SQL Database**: Neon Serverless Postgres
- **AI Provider**: OpenAI API
- **Hosting**: GitHub Pages (frontend), cloud hosting (backend)

## Constitution Check

### Compliance Assessment

- ✅ **Accuracy and Authority**: RAG system will ensure answers are sourced only from textbook content
- ✅ **Clarity and Accessibility**: UI will be designed for intuitive interaction
- ✅ **Reproducibility and Traceability**: All components will be documented with setup instructions
- ✅ **Seamless Integration**: Chatbot will be embedded within textbook interface
- ✅ **Functional Code and Validation**: All code will be tested and validated
- ✅ **Deployability and Reliability**: Deployment will be automated with high availability

### Risk Mitigation

- **Hallucination Prevention**: Strict content filtering and source attribution
- **Performance**: Caching strategies and optimized vector search
- **Scalability**: Serverless architecture with auto-scaling capabilities
- **Security**: Proper authentication and data protection measures

## Research Phase (Phase 0)

### Research Summary

Based on research.md, the following key questions have been addressed:

1. **Textbook Content Structure**: Content is expected to be in markdown format compatible with Docusaurus, with semantic chunking for vector indexing
2. **OpenAI Model Selection**: GPT-4 Turbo recommended for educational Q&A due to reasoning capabilities for technical concepts
3. **API Rate Limits and Costs**: GPT-4 Turbo provides sufficient capacity for educational use with appropriate cost management
4. **Expected User Load**: System designed for 10-50 concurrent users during peak hours
5. **Content Pre-processing**: Textbook content to be chunked into semantic sections with vector embeddings stored in Qdrant

### Best Practices Implementation

1. **RAG Implementation**: Implement source attribution, confidence scoring, and hallucination detection
2. **Vector Database**: Use dense vector embeddings with HNSW indexing for fast retrieval
3. **Session Management**: Implement time-based expiration and conversation context maintenance
4. **Response Attribution**: Clear citation format with links to original content

## Data Model Design (Phase 1)

### Entity Relationships

```
StudentSession
├── id: UUID
├── created_at: DateTime
├── last_activity: DateTime
├── mode: Enum('full_book', 'selected_text')
└── metadata: JSON

Question
├── id: UUID
├── session_id: UUID (foreign key to StudentSession)
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

TextbookContent
├── id: UUID
├── section: String
├── content: Text
├── embedding: Vector
├── metadata: JSON
└── created_at: DateTime
```

## API Contracts (Phase 1)

### REST API Endpoints

#### Session Management
- `POST /api/sessions` - Create new session
- `GET /api/sessions/{session_id}` - Get session details
- `PUT /api/sessions/{session_id}/mode` - Update session mode

#### Question/Answer Interface
- `POST /api/sessions/{session_id}/questions` - Submit question and get answer
- `GET /api/sessions/{session_id}/history` - Get conversation history

#### Content Management
- `GET /api/content/search` - Search textbook content
- `POST /api/content/index` - Index new content (admin only)

### OpenAPI Specification

```yaml
openapi: 3.0.1
info:
  title: RAG Chatbot API
  version: 1.0.0
  description: API for the RAG Chatbot embedded in Physical AI & Humanoid Robotics textbook

paths:
  /api/sessions:
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

  /api/sessions/{session_id}/questions:
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

## Quickstart Guide (Phase 1)

### Development Environment Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Set up frontend (Docusaurus)**
   ```bash
   cd frontend
   npm install
   ```

3. **Set up backend environment**
   ```bash
   # Create virtual environment
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install backend dependencies
   pip install fastapi uvicorn openai python-dotenv qdrant-client asyncpg
   ```

4. **Configure environment variables**
   ```bash
   # Create .env file
   touch .env

   # Add required environment variables:
   OPENAI_API_KEY=your_openai_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_database_url
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

### Initial Development Tasks

1. Set up project structure and dependencies
2. Implement basic FastAPI backend with health check
3. Create database models and setup
4. Implement Qdrant vector storage integration
5. Build basic chatbot functionality
6. Integrate with Docusaurus frontend
7. Implement the two required modes (full book and selected text)
8. Add proper error handling and source attribution

## Re-evaluation Post-Design

### Updated Risk Assessment

- **Data Privacy**: Ensure student interactions are properly anonymized
- **Content Accuracy**: Implement validation mechanisms for response quality
- **Performance**: Optimize vector search and API response times
- **Cost Management**: Implement rate limiting and usage monitoring

### Architecture Validation

The proposed architecture aligns with the constitutional principles:
- Ensures accuracy through strict content sourcing
- Provides accessibility through clear UI/UX
- Maintains traceability with source attribution
- Enables seamless integration with textbook
- Supports reliability with proper error handling