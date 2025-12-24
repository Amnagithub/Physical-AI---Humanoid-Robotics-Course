from pydantic_settings import BaseSettings
from typing import Optional


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

    # RAG settings
    TOP_K: int = 5
    CONFIDENCE_THRESHOLD: float = 0.85
    MAX_RESPONSE_LATENCY: float = 2.0  # seconds

    class Config:
        env_file = ".env"


settings = Settings()