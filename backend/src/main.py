from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api.sessions import router as sessions_router
from .api.questions import router as questions_router
from .config import settings

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

# Include API routers
app.include_router(sessions_router, prefix="/api/v1", tags=["sessions"])
app.include_router(questions_router, prefix="/api/v1", tags=["questions"])

@app.get("/")
def read_root():
    return {"message": "Cohere-based RAG Chatbot API"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "cohere-rag-chatbot-api"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)