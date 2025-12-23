"""
Test script to verify core functionality of the RAG Chatbot
"""
import asyncio
import os
from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService
from src.services.rag_service import RAGService
from src.config import settings

async def test_core_functionality():
    """Test the core functionality of the RAG system"""
    print("Testing RAG Chatbot core functionality...")

    # Initialize services
    try:
        qdrant_service = QdrantService()
        print("✓ Qdrant service initialized")
    except Exception as e:
        print(f"✗ Error initializing Qdrant service: {e}")
        return False

    try:
        embedding_service = EmbeddingService()
        print("✓ Embedding service initialized")
    except Exception as e:
        print(f"✗ Error initializing embedding service: {e}")
        return False

    try:
        rag_service = RAGService(qdrant_service, embedding_service)
        print("✓ RAG service initialized")
    except Exception as e:
        print(f"✗ Error initializing RAG service: {e}")
        return False

    # Test 1: Generate embedding
    test_text = "What is artificial intelligence?"
    try:
        embedding = await embedding_service.generate_embedding(test_text)
        if embedding and len(embedding) > 0:
            print("✓ Embedding generation works")
        else:
            print("✗ Embedding generation failed")
            return False
    except Exception as e:
        print(f"✗ Error during embedding generation: {e}")
        return False

    # Test 2: Content retrieval (this will return empty since no content is ingested yet)
    try:
        results = await rag_service.retrieve_content(test_text)
        print(f"✓ Content retrieval works (found {len(results)} results)")
    except Exception as e:
        print(f"✗ Error during content retrieval: {e}")
        return False

    # Test 3: Response generation (will return "not covered" since no content exists yet)
    try:
        response, sources, confidence = await rag_service.generate_response(test_text)
        print(f"✓ Response generation works")
        print(f"  Response: {response[:100]}...")
        print(f"  Sources: {len(sources)}")
        print(f"  Confidence: {confidence}")
    except Exception as e:
        print(f"✗ Error during response generation: {e}")
        return False

    # Test 4: Selected text mode
    try:
        selected_text = "Artificial intelligence is a branch of computer science."
        response, sources, confidence = await rag_service.generate_response(
            test_text,
            selected_text=selected_text,
            mode='selected_text'
        )
        print(f"✓ Selected text mode works")
        print(f"  Response: {response[:100]}...")
    except Exception as e:
        print(f"✗ Error during selected text mode: {e}")
        return False

    print("\n✓ All core functionality tests passed!")
    return True

if __name__ == "__main__":
    # Check if required environment variables are set
    if not os.getenv("OPENAI_API_KEY"):
        print("Please set OPENAI_API_KEY environment variable")
        exit(1)

    if not os.getenv("QDRANT_URL") and not (os.getenv("QDRANT_HOST") and os.getenv("QDRANT_PORT")):
        print("Please set QDRANT_URL or QDRANT_HOST and QDRANT_PORT environment variables")
        exit(1)

    success = asyncio.run(test_core_functionality())
    if success:
        print("\n🎉 Core functionality verification completed successfully!")
    else:
        print("\n❌ Some tests failed.")
        exit(1)