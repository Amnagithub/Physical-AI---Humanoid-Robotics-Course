# Physical AI & Humanoid Robotics Course RAG Chatbot Specification

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-12-23
**Source**: Physical-AI---Humanoid-Robotics-Course repository

## Problem Statement

The Physical AI & Humanoid Robotics Course needed an embedded chatbot that could answer student questions about the course content using Retrieval-Augmented Generation (RAG). Students needed an intelligent assistant that could provide accurate, context-aware responses based on the course materials, with proper source citations.

## System Intent

**Target Users**: Students taking the Physical AI & Humanoid Robotics Course

**Core Value Proposition**: An AI-powered chatbot embedded in the course documentation that answers student questions using RAG techniques, providing accurate responses with source citations from the course materials.

**Key Capabilities**:
- **Question Answering**: Answer student questions based on course content
- **Session Management**: Track conversation history and context
- **Mode Selection**: Support both full-book and selected-text modes
- **Source Attribution**: Provide citations for information sources
- **Confidence Scoring**: Indicate reliability of responses

## Functional Requirements

### Requirement 1: Create Chat Session
- **What**: Create a new chat session for a student
- **Why**: To track conversation history and maintain context
- **Inputs**: Optional mode ('full_book' or 'selected_text')
- **Outputs**: Session ID and creation timestamp
- **Side Effects**: New session record in database
- **Success Criteria**: Session is created with unique ID and proper mode

### Requirement 2: Submit Question and Get Answer
- **What**: Submit a question and receive an AI-generated answer
- **Why**: To provide students with immediate answers to their questions
- **Inputs**: Session ID, question content, optional selected text, optional mode
- **Outputs**: Answer content, source citations, confidence score, creation timestamp
- **Side Effects**: New question and answer records in database
- **Success Criteria**: Relevant answer with proper source attribution

### Requirement 3: Update Session Mode
- **What**: Change the session's context mode
- **Why**: Allow students to switch between full-book and selected-text modes
- **Inputs**: Session ID, new mode ('full_book' or 'selected_text')
- **Outputs**: Updated session information
- **Side Effects**: Session mode updated in database
- **Success Criteria**: Mode is successfully updated

### Requirement 4: Retrieve Session Information
- **What**: Get details about a specific session
- **Why**: To display session information to the user
- **Inputs**: Session ID
- **Outputs**: Session ID, creation time, mode, last activity time
- **Side Effects**: None
- **Success Criteria**: Session information returned correctly

### Requirement 5: Retrieve Session Questions
- **What**: Get all questions for a specific session
- **Why**: To display conversation history
- **Inputs**: Session ID, optional pagination parameters
- **Outputs**: List of questions with content and metadata
- **Side Effects**: None
- **Success Criteria**: Questions returned in chronological order

## Non-Functional Requirements

### Performance
- **Target**: Response time < 5 seconds for typical questions
- **Concurrent Users**: Support 100+ simultaneous sessions
- **Caching**: Implement Qdrant vector search for fast retrieval

### Security
- **Authentication**: No user authentication required (anonymous sessions)
- **Input Validation**: Validate question content and session IDs
- **Data Privacy**: No personal data collection beyond session information

### Reliability
- **Availability**: 99.9% uptime for educational purposes
- **Error Handling**: Graceful degradation when external services fail
- **Data Persistence**: All conversations persisted in database

### Scalability
- **Horizontal Scaling**: Stateless API design allows multiple instances
- **Database Scaling**: PostgreSQL with connection pooling
- **Vector Database**: Qdrant cloud service for embeddings

### Observability
- **Logging**: Structured logging with request correlation
- **Health Checks**: API health endpoint for monitoring
- **Error Reporting**: Detailed error responses for debugging

## System Constraints

### External Dependencies
- **OpenAI API**: For LLM responses (GPT-4 Turbo or fallback models)
- **Qdrant Cloud**: For vector storage and similarity search
- **PostgreSQL**: Neon database for session and conversation storage
- **Cohere API**: Alternative LLM provider

### Data Formats
- **API Requests/Responses**: JSON over REST
- **Embeddings**: Generated using text-embedding-ada-002 or Cohere models
- **Content Storage**: Textbook content in structured format

### Deployment Context
- **Backend**: FastAPI application deployed to cloud platform
- **Frontend**: Docusaurus static site with React components
- **Environment**: Supports development, staging, and production

### Compliance Requirements
- **GDPR**: Minimal data collection, no personal information
- **Educational Use**: Designed for course content, not general purpose

## Non-Goals & Out of Scope

**Explicitly excluded**:
- General-purpose chatbot (limited to course content)
- Real-time collaboration features
- Advanced user management
- Integration with external LMS systems
- Offline functionality

## Known Gaps & Technical Debt

### Gap 1: Missing Logger in questions.py
- **Issue**: Undefined logger variable on line 110 of questions.py
- **Evidence**: `logger.error(f"Error processing question: {e}\n{traceback.format_exc()}")` without logger import
- **Impact**: Error handling fails when RAG service encounters issues
- **Recommendation**: Add proper logger import and initialization

### Gap 2: Hardcoded API Keys in Config
- **Issue**: API keys visible in config.py
- **Evidence**: Qdrant and other service credentials in source code
- **Impact**: Security risk if repository is public
- **Recommendation**: Use environment variables exclusively

### Gap 3: Missing Content Ingestion Service
- **Issue**: No mechanism to populate textbook content
- **Evidence**: TextbookContent model exists but no ingestion endpoint
- **Impact**: RAG system has no content to retrieve from
- **Recommendation**: Implement content ingestion API

## Success Criteria

### Functional Success
- [ ] All API endpoints return correct responses for valid inputs
- [ ] All error cases handled gracefully
- [ ] All integrations with external systems work correctly
- [ ] RAG system retrieves relevant content when available
- [ ] Session management works correctly

### Non-Functional Success
- [ ] Response time < 5 seconds for 95% of requests
- [ ] System handles 50+ concurrent users
- [ ] 95% test coverage achieved
- [ ] Zero critical security vulnerabilities
- [ ] Proper source attribution in responses

## Acceptance Tests

### Test 1: Create and Use Chat Session
**Given**: User opens course website
**When**: User clicks chat button and asks a question
**Then**: Question is processed and relevant answer is returned with sources

### Test 2: Switch Context Modes
**Given**: User has active chat session
**When**: User switches from 'full_book' to 'selected_text' mode
**Then**: Subsequent questions use selected text context

### Test 3: Handle Unknown Topics
**Given**: User asks about topic not in course materials
**When**: Question is submitted
**Then**: System responds that information is not covered in the book