# Feature Specification: Embedded RAG Chatbot for a Published Book

**Feature Branch**: `1-rag-chatbot`
**Created**: 2025-12-24
**Status**: Draft
**Input**: User description: "Project

Embedded RAG Chatbot for a Published Book

Goal

Build a Cohere-powered Retrieval-Augmented Generation (RAG) chatbot embedded in a published book, capable of answering questions only from book content or explicitly selected text.

Target Users

Readers, researchers, and educators requiring precise, text-grounded answers.

Required Stack

LLM & Embeddings: Cohere

API: FastAPI

Vector DB: Qdrant Cloud (Free Tier)

Relational DB: Neon Serverless Postgres

Prompt Governance: SpecifyKit Plus

Execution: Qween CLI

Disallowed: OpenAI APIs or SDKs

Credential Policy

All secrets must be injected via environment variables at runtime:

COHERE_API_KEY
QDRANT_ENDPOINT
QDRANT_API_KEY
QDRANT_CLUSTER_ID
NEON_DATABASE_URL


No credentials in prompts or source code.

Functional Scope (Build)

Book content ingestion & chunking

Cohere embeddings stored in Qdrant

Semantic retrieval (Top-K ≤ 5)

Context-bounded answer generation

Selected-text-only answer mode

Embeddable chatbot API

Out of Scope

External knowledge or web search

Multi-book reasoning

Model fine-tuning

Ethical or product comparisons

Behavioral Constraints

Answers must be strictly grounded in retrieved or selected text

No speculation or hallucination

Refuse if context is insufficient

Refusal message (exact):

“The provided text does not contain sufficient information to answer this question.”

Quality Targets

Retrieval relevance ≥ 0.85

Latency ≤ 2s

Zero hallucination tolerance

Success Criteria

Accurate book-grounded answers

Selected text overrides global retrieval

Fully deployable via Qween CLI

Production-safe, reproducible behavior"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions about Book Content (Priority: P1)

A reader, researcher, or educator reading a published book wants to ask questions about specific concepts they don't understand. They interact with the integrated chatbot to get answers based only on the book content or explicitly selected text.

**Why this priority**: This is the core functionality that provides immediate value by helping users understand complex concepts from the book with precise, text-grounded answers.

**Independent Test**: User can ask a question about any topic in the book and receive an accurate answer that is directly sourced from the book content, with no external information or hallucinations.

**Acceptance Scenarios**:

1. **Given** user is viewing the book content, **When** they ask a question about a concept in the book, **Then** the chatbot provides an accurate answer based solely on the book content.

2. **Given** user asks a question not covered in the book, **When** they submit the query, **Then** the chatbot responds with "The provided text does not contain sufficient information to answer this question."

---

### User Story 2 - Contextual Question Answering with Selected Text (Priority: P2)

A user selects specific text from the book and wants to ask questions specifically about that selected content. They use the "Selected Text Only" mode to get focused answers based only on their selection.

**Why this priority**: This provides an advanced feature for users who want to dive deeper into specific passages they're currently reading, with answers strictly limited to the selected text.

**Independent Test**: User can select text, ask a question about it, and receive answers that are exclusively based on the selected text, not the broader book content.

**Acceptance Scenarios**:

1. **Given** user has selected text from the book, **When** they ask a question in "Selected Text Only" mode, **Then** the chatbot responds using only information from the selected text or replies "The provided text does not contain sufficient information to answer this question."

---

### User Story 3 - Book Content Ingestion and Indexing (Priority: P3)

The system needs to process and index the entire book content to enable semantic retrieval of relevant passages when users ask questions.

**Why this priority**: This foundational capability enables the RAG functionality by creating searchable embeddings of the book content using Cohere technology.

**Independent Test**: The system can ingest book content, chunk it appropriately, generate Cohere embeddings, and store them in Qdrant for semantic retrieval.

**Acceptance Scenarios**:

1. **Given** book content is provided, **When** the ingestion process runs, **Then** the content is properly chunked and Cohere embeddings are stored in Qdrant.

2. **Given** content is indexed, **When** a user query is processed, **Then** the system retrieves up to 5 relevant chunks (Top-K ≤ 5) with relevance ≥ 0.85.

---

### Edge Cases

- What happens when retrieval confidence is below 0.85 threshold?
- How does the system handle ambiguous questions that could refer to multiple concepts?
- What happens when the selected text mode is used but no text is selected?
- How does the system handle very long or very short questions?
- What happens when the system is under high load and responses are delayed?
- How does the system handle credentials when environment variables are missing?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide answers based solely on the provided book content or explicitly selected text
- **FR-002**: System MUST respond with "The provided text does not contain sufficient information to answer this question." when asked questions not addressed in the provided context
- **FR-003**: System MUST support two modes: Full Book RAG mode (default) and Selected Text Only mode (strict)
- **FR-004**: System MUST use Cohere for LLM and embeddings as the authorized technology
- **FR-005**: System MUST store embeddings in Qdrant Cloud (Free Tier) for semantic retrieval
- **FR-006**: System MUST use Neon Serverless Postgres for relational database needs
- **FR-007**: System MUST retrieve Top-K chunks with K ≤ 5 for context-bounded generation
- **FR-008**: System MUST ensure retrieval confidence ≥ 0.85 for valid responses
- **FR-009**: System MUST respond with latency ≤ 2s for user queries
- **FR-010**: System MUST have zero hallucination tolerance
- **FR-011**: System MUST be deployable via Qween CLI
- **FR-012**: System MUST NOT use OpenAI APIs or SDKs (strictly disallowed)
- **FR-013**: System MUST accept credentials only via environment variables at runtime
- **FR-014**: System MUST NOT store credentials in prompts or source code

### Key Entities

- **User Session**: Represents a user's interaction session with the chatbot, including conversation history and context
- **Question**: A query submitted by the user to the chatbot
- **Answer**: The response generated by the RAG system based on book content
- **Book Content**: The published book data that serves as the knowledge base
- **Text Selection**: A portion of text selected by the user for the "Selected Text Only" mode
- **Embeddings**: Cohere-generated vector representations of book content stored in Qdrant
- **Credentials**: Runtime environment variables including COHERE_API_KEY, QDRANT_ENDPOINT, QDRANT_API_KEY, QDRANT_CLUSTER_ID, and NEON_DATABASE_URL

## Assumptions

- Users have access to the digital version of the published book
- The book content is properly indexed and searchable for the RAG system
- Users will access the chatbot feature through the same interface as the book
- Network connectivity is available for the AI system to process queries
- The book content is comprehensive enough to answer most user questions
- Required environment variables will be properly configured at runtime
- Cohere API and Qdrant Cloud will be available during operation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can get accurate answers to book-related questions within 2 seconds of submission
- **SC-002**: 95% of user questions about book content receive responses that are directly sourced from the book
- **SC-003**: Retrieval relevance is ≥ 0.85 for all responses
- **SC-004**: Zero percent of responses contain hallucinations or information not found in the provided text
- **SC-005**: Users can successfully use both Full Book RAG and Selected Text Only modes as intended
- **SC-006**: System is fully deployable via Qween CLI with production-safe, reproducible behavior
- **SC-007**: Selected text mode correctly overrides global retrieval when activated
- **SC-008**: System handles external knowledge or web search requests by refusing to answer with the specified refusal message