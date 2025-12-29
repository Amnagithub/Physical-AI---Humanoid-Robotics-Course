#!/usr/bin/env python3
"""
Script to ingest course documentation into Qdrant for RAG chatbot.
"""

import os
import sys
from pathlib import Path

# Add backend to path
backend_path = Path(__file__).parent.parent
sys.path.insert(0, str(backend_path))

from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from tqdm import tqdm

# Load environment
from dotenv import load_dotenv
load_dotenv(backend_path / ".env")

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
COLLECTION_NAME = "book_content"

def read_markdown_files(docs_dir: str) -> dict:
    """Read all markdown files from the docs directory."""
    content = {}
    docs_path = Path(docs_dir)

    for md_file in tqdm(list(docs_path.rglob("*.md")), desc="Reading files"):
        try:
            relative_path = md_file.relative_to(docs_path)
            with open(md_file, 'r', encoding='utf-8') as f:
                content[str(relative_path)] = f.read()
        except Exception as e:
            print(f"Warning: Could not read {md_file}: {e}")

    return content

def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> list:
    """Simple chunking of text by sentences."""
    import re
    sentences = re.split(r'(?<=[.!?])\s+', text)
    chunks = []
    current_chunk = []

    for sentence in sentences:
        if len(' '.join(current_chunk)) + len(sentence) > chunk_size:
            if current_chunk:
                chunks.append(' '.join(current_chunk))
            current_chunk = [sentence]
        else:
            current_chunk.append(sentence)

    if current_chunk:
        chunks.append(' '.join(current_chunk))

    return chunks

def generate_embeddings(texts: list, cohere_client) -> list:
    """Generate embeddings using Cohere in batches."""
    all_embeddings = []
    batch_size = 90  # Slightly under the 96 limit

    for i in tqdm(range(0, len(texts), batch_size), desc="Generating embeddings"):
        batch = texts[i:i + batch_size]
        response = cohere_client.embed(
            texts=batch,
            model="embed-english-v3.0",
            input_type="search_document",
            embedding_types=["float"]
        )
        all_embeddings.extend(response.embeddings.float_)

    return all_embeddings

def main():
    print("=" * 60)
    print("Course Documentation Ingestion Script")
    print("=" * 60)

    # Initialize clients
    print("\n1. Connecting to Qdrant...")
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    print("   ✓ Connected to Qdrant")

    print("\n2. Initializing Cohere client...")
    cohere_client = cohere.ClientV2(api_key=COHERE_API_KEY)
    print("   ✓ Connected to Cohere")

    # Create collection if it doesn't exist
    print("\n3. Ensuring collection exists...")
    collections = qdrant_client.get_collections().collections
    collection_names = [c.name for c in collections]

    if COLLECTION_NAME not in collection_names:
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )
        print(f"   ✓ Created collection '{COLLECTION_NAME}'")
    else:
        print(f"   ✓ Collection '{COLLECTION_NAME}' already exists")

    # Read markdown files
    docs_dir = Path(__file__).parent.parent.parent / "docs"
    print(f"\n4. Reading markdown files from: {docs_dir}")
    content = read_markdown_files(str(docs_dir))
    print(f"   ✓ Found {len(content)} files")

    if not content:
        print("   ✗ No markdown files found!")
        return

    # Chunk content
    print("\n5. Chunking content...")
    all_chunks = []
    all_metadata = []

    for filepath, text in content.items():
        chunks = chunk_text(text)
        for i, chunk in enumerate(chunks):
            all_chunks.append(chunk)
            all_metadata.append({
                "source": filepath,
                "chunk_index": i,
                "total_chunks": len(chunks)
            })

    print(f"   ✓ Created {len(all_chunks)} chunks")

    # Generate embeddings
    print("\n6. Generating embeddings (this may take a few minutes)...")
    embeddings = generate_embeddings(all_chunks, cohere_client)
    print(f"   ✓ Generated {len(embeddings)} embeddings")

    # Upload to Qdrant
    print("\n7. Uploading to Qdrant...")
    points = [
        models.PointStruct(
            id=i,
            vector=embeddings[i],
            payload={
                "content": all_chunks[i],
                "metadata": all_metadata[i]
            }
        )
        for i in range(len(all_chunks))
    ]

    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )
    print(f"   ✓ Uploaded {len(points)} points to Qdrant")

    # Get final count
    count = qdrant_client.count(
        collection_name=COLLECTION_NAME,
        exact=True
    )
    print(f"\n   Total vectors in collection: {count.count}")

    print("\n" + "=" * 60)
    print("Ingestion complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()
