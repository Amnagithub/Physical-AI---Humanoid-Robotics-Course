import os
from typing import Optional
from pydantic import Field, field_validator, FieldValidationInfo
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Application settings
    app_name: str = "RAG Chatbot API"
    app_version: str = "1.0.0"
    app_env: str = Field(default="development", env="APP_ENV")
    debug: bool = Field(default=False, env="DEBUG")
    log_level: str = Field(default="info", env="LOG_LEVEL")

    # API settings
    api_v1_prefix: str = "/api/v1"
    max_content_size: int = Field(default=1048576, env="MAX_CONTENT_SIZE")  # 1MB

    # OpenAI settings
    openai_api_key: Optional[str] = Field(default=None, env="OPENAI_API_KEY")  # Optional for fallback
    openai_model: str = Field(default="gpt-4-turbo-preview", env="OPENAI_MODEL")

    # OpenRouter settings
    openrouter_api_key: Optional[str] = Field(default=None, env="OPENROUTER_API_KEY")  # Optional for fallback
    openrouter_model: str = Field(default="qwen/qwen-2-72b-instruct", env="OPENROUTER_MODEL")
    openrouter_embedding_model: str = Field(default="nomic-ai/nomic-embed-text-v1.5", env="OPENROUTER_EMBEDDING_MODEL")

    # Embedding model selection
    embedding_model: str = Field(default="text-embedding-ada-002", env="EMBEDDING_MODEL")

    # LLM selection
    llm_provider: str = Field(default="openrouter", env="LLM_PROVIDER")  # openai or openrouter

    def model_post_init(self, __context):
        # Validate that at least one API key is provided based on the selected provider
        if self.llm_provider == "openrouter" and not self.openrouter_api_key:
            raise ValueError("OPENROUTER_API_KEY is required when using openrouter provider")
        elif self.llm_provider == "openai" and not self.openai_api_key:
            raise ValueError("OPENAI_API_KEY is required when using openai provider")
        elif self.llm_provider not in ["openrouter", "openai"]:
            raise ValueError("LLM_PROVIDER must be either 'openrouter' or 'openai'")

    # Database settings
    database_url: str = Field(default="postgresql+asyncpg://username:password@localhost/dbname", env="DATABASE_URL")
    neon_db_url: Optional[str] = Field(default=None, env="NEON_DB_URL")

    # Qdrant settings
    qdrant_url: Optional[str] = Field(default=None, env="QDRANT_URL")
    qdrant_api_key: Optional[str] = Field(default=None, env="QDRANT_API_KEY")
    qdrant_host: str = Field(default="localhost", env="QDRANT_HOST")
    qdrant_port: int = Field(default=6333, env="QDRANT_PORT")

    # Session settings
    session_timeout: int = Field(default=86400, env="SESSION_TIMEOUT")  # 24 hours in seconds
    default_mode: str = Field(default="full_book", env="DEFAULT_MODE")  # 'full_book' or 'selected_text'

    # RAG settings
    rag_max_sources: int = Field(default=5, env="RAG_MAX_SOURCES")
    rag_search_limit: int = Field(default=10, env="RAG_SEARCH_LIMIT")
    min_confidence_score: float = Field(default=0.3, env="MIN_CONFIDENCE_SCORE")

    class Config:
        env_file = ".env"
        case_sensitive = True
        extra = "ignore"  # Allow extra fields from .env


# Create a global settings instance
settings = Settings()

# Helper function to get environment-specific configuration
def get_environment_config():
    env = settings.app_env.lower()

    if env == "production":
        return {
            "log_level": "warning",
            "debug": False,
            "database_pool_size": 20,
            "max_workers": 10
        }
    elif env == "staging":
        return {
            "log_level": "info",
            "debug": False,
            "database_pool_size": 10,
            "max_workers": 5
        }
    else:  # development
        return {
            "log_level": "debug",
            "debug": True,
            "database_pool_size": 5,
            "max_workers": 2
        }


# Get the current environment configuration
env_config = get_environment_config()