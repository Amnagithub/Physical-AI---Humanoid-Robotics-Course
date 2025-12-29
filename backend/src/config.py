from pydantic_settings import BaseSettings
from typing import Optional
from pathlib import Path

# Get the backend directory (parent of src)
BACKEND_DIR = Path(__file__).resolve().parent.parent


class Settings(BaseSettings):
    # Cohere settings
    COHERE_API_KEY: str

    # Qdrant settings
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_CLUSTER_ID: Optional[str] = None
    QDRANT_COLLECTION_NAME: str = "book_content"

    # Database settings
    NEON_DATABASE_URL: str

    # Application settings
    APP_NAME: str = "Cohere-based RAG Chatbot"
    DEBUG: bool = False

    # RAG settings - lowered threshold for better recall
    TOP_K: int = 10
    CONFIDENCE_THRESHOLD: float = 0.1  # Lowered from 0.5 to get more results
    MAX_RESPONSE_LATENCY: float = 5.0  # Increased to 5s for slower queries

    class Config:
        env_file = BACKEND_DIR / ".env"


settings = Settings()