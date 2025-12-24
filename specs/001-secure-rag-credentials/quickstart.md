# Quickstart Guide: Secure RAG Credentials Integration

## Prerequisites
- Python 3.8+
- pip package manager
- Access to Cohere API
- Access to Qdrant Cloud
- Access to Neon Serverless Postgres

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r backend/requirements.txt
```

### 4. Configure Environment Variables
Create a `.env` file in the backend directory with the following variables:

```bash
# Cohere Configuration
COHERE_API_KEY=vjH3RkTSzqxySgPUfFVVTOD49NGXhEhqe95VZTaS

# Qdrant Configuration
QDRANT_URL=https://95118e17-e14a-43f4-bb0d-8b514273d734.europe-west3-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.5o9rCJgjSIWkVw_WztpA-ZL5BAZqgrokjkKeL9EEcXQ

# Neon Postgres Configuration
NEON_DATABASE_URL=postgresql://neondb_owner:npg_t0PBTbWGa5Rp@ep-withered-fire-a42ip3ex-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# Application Configuration
APP_ENV=development
DEBUG=false
LOG_LEVEL=info
```

### 5. Run the Application
```bash
cd backend
python -m uvicorn src.main:app --reload
```

The application will validate all credentials on startup and fail with clear error messages if any required credentials are missing or invalid.

## Verification Steps
1. Check that the application starts without credential-related errors
2. Verify that the application logs show successful validation of all credentials
3. Confirm that the API is accessible at the specified port