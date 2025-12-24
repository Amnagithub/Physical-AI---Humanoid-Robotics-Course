import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

from backend.src.services.rag_service import RAGService
from backend.src.config import settings
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def test_backend_structure():
    """
    Test that the backend structure is properly set up
    """
    print("Testing backend structure...")

    # Test that we can import and instantiate the RAG service
    try:
        rag_service = RAGService()
        print("✓ RAGService instantiated successfully")
    except Exception as e:
        print(f"✗ Failed to instantiate RAGService: {e}")
        return False

    # Test that environment variables are properly configured
    required_vars = [
        'COHERE_API_KEY',
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'NEON_DATABASE_URL'
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"⚠ Missing environment variables: {missing_vars}")
        print("  Please set these in your .env file")
    else:
        print("✓ All required environment variables are set")

    # Test that settings are loaded properly
    try:
        settings_instance = settings
        print("✓ Settings loaded successfully")
    except Exception as e:
        print(f"✗ Failed to load settings: {e}")
        return False

    print("\nBackend structure test completed!")
    return True

if __name__ == "__main__":
    test_backend_structure()