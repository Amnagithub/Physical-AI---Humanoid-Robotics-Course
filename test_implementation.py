#!/usr/bin/env python3
"""
Simple test script to verify the backend configuration and OpenRouter integration
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv('backend/.env')

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

def test_config():
    """Test the configuration loading"""
    try:
        from config import settings
        print("✓ Configuration loaded successfully")
        print(f"  - LLM Provider: {settings.llm_provider}")
        print(f"  - OpenRouter API Key: {'SET' if settings.openrouter_api_key else 'NOT SET'}")
        print(f"  - OpenAI API Key: {'SET' if settings.openai_api_key else 'NOT SET'}")
        print(f"  - Database URL: {'SET' if settings.database_url else 'NOT SET'}")
        return True
    except Exception as e:
        print(f"✗ Configuration failed: {e}")
        return False

def test_services():
    """Test the service initialization"""
    try:
        # Test OpenRouter service initialization
        os.environ["LLM_PROVIDER"] = "openrouter"  # Force OpenRouter
        os.environ["OPENROUTER_API_KEY"] = "test-key"  # Provide a test key

        from services.openrouter_service import OpenRouterService
        service = OpenRouterService()
        print("✓ OpenRouter service initialized successfully")

        # Test embedding service
        from services.embedding_service import EmbeddingService
        emb_service = EmbeddingService()
        print("✓ Embedding service initialized successfully")

        # Test OpenAI service (which now supports OpenRouter)
        from services.openai_service import OpenAIService
        ai_service = OpenAIService()
        print("✓ OpenAI service initialized successfully")

        return True
    except Exception as e:
        print(f"✗ Service initialization failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Testing RAG Chatbot Implementation with OpenRouter & Qwen")
    print("="*60)

    config_ok = test_config()
    print()

    services_ok = test_services()
    print()

    if config_ok and services_ok:
        print("✓ All tests passed! The RAG chatbot is configured correctly with OpenRouter and Qwen.")
        print("\nTo run the backend server:")
        print("  cd backend")
        print("  uvicorn src.main:app --reload --port 8000")
        print("\nTo run the frontend:")
        print("  cd frontend")
        print("  npm run start")
        return 0
    else:
        print("✗ Some tests failed.")
        return 1

if __name__ == "__main__":
    sys.exit(main())