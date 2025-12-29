# Physical AI & Humanoid Robotics Course

An interactive educational platform for learning Physical AI and Humanoid Robotics with AI-powered features including a RAG chatbot, content personalization, and Urdu translation.

## Demo Video

Watch the platform demo: [Physical AI Course Demo](https://docs.google.com/videos/d/19j-sYPsNlu-BGK1AhUwIXyuiqQiPFnWboBDqKgLOSao/edit?usp=sharing)

## Features

### 🤖 AI Chatbot
- **RAG-powered Q&A**: Ask questions about course content and get accurate answers
- **Cohere Integration**: Uses state-of-the-art embeddings and LLM for semantic search
- **Source Attribution**: Every answer includes references to source material
- **Confidence Scoring**: Shows confidence score for each response

### 👤 User Authentication
- **Better Auth**: Secure authentication with email/password
- **Session Management**: Persistent sessions across devices
- **Profile Management**: Set software and hardware background levels

### 📚 Content Personalization
- **Adaptive Learning**: Content adapts to your background level (beginner/intermediate/advanced)
- **Personalized Explanations**: Simplified explanations for beginners, advanced details for experts
- **Hardware Context**: Relevant hardware examples based on your experience

### 🌐 Urdu Translation
- **Instant Translation**: Translate any chapter content to Urdu
- **Technical Term Preservation**: ROS, Python, and other technical terms kept in English
- **RTL Support**: Full right-to-left Urdu text support

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Frontend (Docusaurus)                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  Auth Forms │  │  Chatbot    │  │  Chapter Personalization│  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Auth Server (Node.js)                        │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Better Auth + Hono + PostgreSQL                         │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Backend API (Python)                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  RAG Service│  │ Personalize │  │  Translation Service    │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                    │
              ┌─────────────────────┴─────────────────────┐
              ▼                                           ▼
┌─────────────────────────┐                 ┌─────────────────────────┐
│  Cohere API             │                 │  Qdrant Cloud           │
│  - Embeddings           │                 │  - Vector Database      │
│  - Chat (Command A)     │                 │  - Semantic Search      │
└─────────────────────────┘                 └─────────────────────────┘
```

## Tech Stack

| Layer | Technology |
|-------|------------|
| **Frontend** | Docusaurus (React) |
| **Auth Server** | Node.js + Hono + Better Auth |
| **Backend API** | Python + FastAPI |
| **Database** | PostgreSQL (Neon) |
| **Vector Database** | Qdrant Cloud |
| **AI/ML** | Cohere API (Embed + Chat) |
| **Documentation** | Markdown + Docusaurus |

## Project Structure

```
Physical-AI---Humanoid-Robotics-Course/
├── auth-server/          # Authentication server (Node.js)
│   ├── src/              # Auth source code
│   └── scripts/          # Database migrations
├── backend/              # FastAPI backend (Python)
│   ├── src/
│   │   ├── api/          # REST endpoints
│   │   ├── models/       # SQLAlchemy models
│   │   ├── repositories/ # Data access layer
│   │   ├── schemas/      # Pydantic schemas
│   │   ├── services/     # Business logic
│   │   └── config.py     # Configuration
│   └── scripts/          # Ingestion scripts
├── frontend/             # Docusaurus documentation site
│   ├── src/
│   │   ├── components/   # React components
│   │   ├── context/      # React context providers
│   │   └── lib/          # Utility libraries
│   └── build/            # Production build
├── docs/                 # Course documentation (markdown)
├── history/              # Prompt History Records (PHRs)
└── specs/                # Feature specifications
```

## Getting Started

### Prerequisites

- Node.js >= 18.0
- Python 3.11+
- PostgreSQL database (Neon)
- Qdrant Cloud account
- Cohere API key

### 1. Clone and Install

```bash
# Clone the repository
git clone https://github.com/yourusername/Physical-AI---Humanoid-Robotics-Course.git
cd Physical-AI---Humanoid-Robotics-Course

# Install auth server dependencies
cd auth-server
npm install

# Install backend dependencies
cd ../backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

**Auth Server (.env):**
```env
DATABASE_URL=postgresql://user:pass@host/db
AUTH_SECRET=your-secret-key
PORT=3001
```

**Backend (.env):**
```env
COHERE_API_KEY=your-cohere-key
QDRANT_URL=https://your-cluster.qdrant.cloud
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION_NAME=book_content
NEON_DATABASE_URL=postgresql://user:pass@host/db
```

### 3. Run Database Migrations

```bash
cd auth-server
node scripts/migrate.js
```

### 4. Start the Services

```bash
# Terminal 1: Auth Server (port 3001)
cd auth-server
npm run dev

# Terminal 2: Backend API (port 8000)
cd backend
python3 -m uvicorn src.main:app --reload

# Terminal 3: Frontend (port 3000)
cd frontend
npm start
```

### 5. Ingest Documentation

```bash
cd backend
python3 scripts/ingest_docs.py
```

## API Endpoints

### Authentication
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/auth/sign-up/email` | Sign up with email |
| POST | `/api/auth/sign-in/email` | Sign in with email |
| GET | `/api/auth/get-session` | Get current session |
| POST | `/api/auth/sign-out` | Sign out |

### Chatbot
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/v1/sessions` | Create chat session |
| POST | `/api/v1/sessions/{id}/questions` | Ask a question |
| GET | `/api/v1/sessions/{id}/history` | Get conversation history |

### Content Personalization
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/v1/content/personalize` | Personalize content for user |
| POST | `/api/v1/content/translate` | Translate content to Urdu |
| GET | `/api/v1/content/user-preferences` | Get user preferences |

## Configuration

### Confidence Threshold
The RAG chatbot uses a confidence threshold (default: 0.1) to filter relevant content. Lower values return more results but may include less relevant content.

### Content Chunking
Documents are chunked into ~512 character segments with overlap for better retrieval.

### Personalization Levels
- **Beginner**: Simple analogies, step-by-step explanations
- **Intermediate**: Standard technical explanations with tips
- **Advanced**: Performance considerations, implementation details

## Development

### Running Tests
```bash
# Backend tests
cd backend
pytest

# Auth server tests
cd auth-server
npm test
```

### Building Frontend
```bash
cd frontend
npm run build
```

### Database Migrations
```bash
cd auth-server
node scripts/migrate.js
```

## License

MIT License

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Support

For issues and questions, please open a GitHub issue.
