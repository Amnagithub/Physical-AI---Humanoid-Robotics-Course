from fastapi import FastAPI
from pydantic import BaseModel
from datetime import datetime
import os

from .api import sessions, questions

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot embedded in Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Include API routes
app.include_router(sessions.router, prefix="/api/v1")
app.include_router(questions.router, prefix="/api/v1")

class HealthCheck(BaseModel):
    """Response model for health check endpoint"""
    status: str
    timestamp: str
    environment: str

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running"}

@app.get("/health", response_model=HealthCheck)
def health_check():
    return HealthCheck(
        status="healthy",
        timestamp=datetime.now().isoformat(),
        environment=os.getenv("APP_ENV", "development")
    )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)