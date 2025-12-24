# Cohere-based RAG Chatbot Backend

This is the backend service for the Cohere-powered Retrieval-Augmented Generation (RAG) chatbot embedded in published books.

## Features

- **Cohere-powered**: Uses Cohere API for LLM and embeddings
- **FastAPI**: Modern, fast web framework for building APIs
- **Qdrant Cloud**: Vector database for semantic search
- **Neon Postgres**: Serverless Postgres for session management
- **Two Modes**:
  - Full Book RAG (default) - answers using entire book content
  - Selected Text Only (strict) - answers using only user-selected text
- **Zero Hallucination Tolerance**: Strict content filtering with exact refusal message

## Architecture

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

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Create a `.env` file with the following environment variables:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_CLUSTER_ID=your_qdrant_cluster_id  # Optional
   NEON_DATABASE_URL=your_neon_database_url
   ```

3. Run the application:
   ```bash
   uvicorn src.main:app --reload
   ```

## API Endpoints

### Session Management
- `POST /api/v1/sessions` - Create new session
- `GET /api/v1/sessions/{session_id}` - Get session details
- `PUT /api/v1/sessions/{session_id}/mode` - Update session mode

### Question/Answer Interface
- `POST /api/v1/sessions/{session_id}/questions` - Submit question and get answer
- `GET /api/v1/sessions/{session_id}/history` - Get conversation history

### Content Management
- `GET /api/v1/content/search` - Search book content
- `POST /api/v1/content/index` - Index new content (admin only)

## Quality Requirements

- Retrieval relevance ≥ 0.85
- Response latency ≤ 2s
- Zero hallucination tolerance
- Exact refusal message: "The provided text does not contain sufficient information to answer this question."