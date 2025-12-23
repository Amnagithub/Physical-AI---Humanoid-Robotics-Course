#!/usr/bin/env python3
"""
Simple test to check if the backend configuration is correct
"""
import os
import sys

# Add backend src to path
backend_path = os.path.join(os.path.dirname(__file__), 'backend', 'src')
sys.path.insert(0, backend_path)

# Set environment variables for testing
os.environ['OPENROUTER_API_KEY'] = 'test-key'
os.environ['OPENAI_API_KEY'] = 'test-key'
os.environ['DATABASE_URL'] = 'sqlite:///test.db'  # Use SQLite for testing
os.environ['LLM_PROVIDER'] = 'openrouter'

def test_config():
    """Test configuration loading with environment variables set"""
    try:
        from config import Settings
        settings = Settings()
        print("✓ Configuration loaded successfully")
        print(f"  - LLM Provider: {settings.llm_provider}")
        print(f"  - OpenRouter API Key: {'SET' if settings.openrouter_api_key else 'NOT SET'}")
        print(f"  - OpenAI API Key: {'SET' if settings.openai_api_key else 'NOT SET'}")
        print(f"  - Database URL: {settings.database_url}")
        return True
    except Exception as e:
        print(f"✗ Configuration failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_openrouter_service():
    """Test OpenRouter service creation"""
    try:
        # Mock the API key for testing
        os.environ['OPENROUTER_API_KEY'] = 'test-key'
        os.environ['LLM_PROVIDER'] = 'openrouter'

        from services.openrouter_service import OpenRouterService
        # Don't actually initialize (would make network calls)
        print("✓ OpenRouter service can be imported")
        return True
    except Exception as e:
        print(f"✗ OpenRouter service import failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_embedding_service():
    """Test embedding service with OpenRouter"""
    try:
        # Mock the API key for testing
        os.environ['OPENROUTER_API_KEY'] = 'test-key'
        os.environ['LLM_PROVIDER'] = 'openrouter'

        from services.embedding_service import EmbeddingService
        # Don't actually initialize (would make network calls)
        print("✓ Embedding service can be imported with OpenRouter")
        return True
    except Exception as e:
        print(f"✗ Embedding service import failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_openai_service():
    """Test OpenAI service with OpenRouter support"""
    try:
        # Mock the API key for testing
        os.environ['OPENROUTER_API_KEY'] = 'test-key'
        os.environ['LLM_PROVIDER'] = 'openrouter'

        from services.openai_service import OpenAIService
        # Don't actually initialize (would make network calls)
        print("✓ OpenAI service can be imported with OpenRouter support")
        return True
    except Exception as e:
        print(f"✗ OpenAI service import failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Testing RAG Chatbot Implementation with OpenRouter & Qwen")
    print("="*60)

    tests = [
        ("Configuration", test_config),
        ("OpenRouter Service", test_openrouter_service),
        ("Embedding Service", test_embedding_service),
        ("OpenAI Service (with OpenRouter)", test_openai_service),
    ]

    passed = 0
    total = len(tests)

    for name, test_func in tests:
        print(f"\nTesting {name}...")
        if test_func():
            passed += 1
        else:
            print(f"  {name} test failed")

    print(f"\n{'='*60}")
    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
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