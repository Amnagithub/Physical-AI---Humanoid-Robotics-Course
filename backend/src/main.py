from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from .api.sessions import router as sessions_router
from .api.questions import router as questions_router
from .api.personalization import router as personalization_router
from .api.ingestion import router as ingestion_router
from .config import settings
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Cohere-based RAG Chatbot API",
    description="API for the Cohere-powered RAG Chatbot embedded in published books",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request logging middleware
@app.middleware("http")
async def log_requests(request: Request, call_next):
    logger.info(f"📥 Incoming request: {request.method} {request.url.path}")
    logger.info(f"   Headers: {dict(request.headers)}")

    response = await call_next(request)

    logger.info(f"   Response status: {response.status_code}")
    return response

# Include API routers
app.include_router(sessions_router, prefix="/api/v1", tags=["sessions"])
app.include_router(questions_router, prefix="/api/v1", tags=["questions"])
app.include_router(personalization_router, prefix="/api/v1", tags=["personalization"])
app.include_router(ingestion_router, prefix="/api/v1", tags=["ingestion"])

@app.get("/")
def read_root():
    logger.info("Health check - API is running")
    return {"message": "Cohere-based RAG Chatbot API"}

@app.get("/health")
def health_check():
    logger.info("Health check endpoint called")
    return {"status": "healthy", "service": "cohere-rag-chatbot-api"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)