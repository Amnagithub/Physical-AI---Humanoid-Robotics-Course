# Physical AI & Humanoid Robotics Course - RAG Chatbot Development Guidelines
Auto-generated from all feature plans. Last updated: 2025-12-22

## Active Technologies

- **Frontend Framework**: Docusaurus (existing)
- **Backend Framework**: FastAPI
- **AI SDK**: OpenAI Agents/ChatKit SDKs
- **Vector Database**: Qdrant Cloud
- **SQL Database**: Neon Serverless Postgres
- **AI Provider**: OpenAI API
- **Hosting**: GitHub Pages (frontend), cloud hosting (backend)

## Project Structure

```text
specs/1-rag-chatbot/
├── spec.md
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
└── contracts/
    └── rag-chatbot-api.yaml
frontend/
├── docusaurus.config.js
├── package.json
├── src/
└── static/
```

## Commands

### Development Setup
```bash
# Backend setup
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install fastapi uvicorn openai python-dotenv qdrant-client asyncpg

# Frontend setup
cd frontend
npm install

# Environment variables
# OPENAI_API_KEY=your_openai_api_key
# QDRANT_URL=your_qdrant_cloud_url
# QDRANT_API_KEY=your_qdrant_api_key
# DATABASE_URL=your_neon_database_url
```

### Running the Application
```bash
# Terminal 1: Start the backend
cd backend
uvicorn main:app --reload

# Terminal 2: Start the frontend
cd frontend
npm start
```

## Code Style

### Python (Backend)
- Use FastAPI for API endpoints
- Follow PEP 8 guidelines
- Use type hints for all function parameters and return values
- Use async/await for I/O operations

### JavaScript/TypeScript (Frontend)
- Follow existing Docusaurus patterns
- Use React best practices
- Maintain consistency with existing codebase

## Recent Changes

- **RAG Chatbot Feature**: Added OpenAI-based chatbot with textbook content retrieval
- **Data Models**: Created StudentSession, Question, Answer, and TextbookContent entities
- **API Contracts**: Defined REST API for session management and Q&A functionality

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->