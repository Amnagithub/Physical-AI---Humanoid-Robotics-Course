import asyncio
from typing import List, Dict, Any
from ..services.embedding_service import EmbeddingService
from ..services.qdrant_service import QdrantService


class ContentIngestionService:
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()

    def chunk_text(self, text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
        """
        Split text into overlapping chunks
        """
        chunks = []
        start = 0
        text_length = len(text)

        while start < text_length:
            end = start + chunk_size
            chunk = text[start:end]

            # Ensure we don't split in the middle of a sentence if possible
            if end < text_length:
                # Look for sentence boundaries
                last_period = chunk.rfind('.')
                last_exclamation = chunk.rfind('!')
                last_question = chunk.rfind('?')
                last_boundary = max(last_period, last_exclamation, last_question)

                if last_boundary > chunk_size // 2:  # Only adjust if we're not cutting too early
                    end = start + last_boundary + 1
                    chunk = text[start:end]

            chunks.append(chunk)
            start = end - overlap

        return chunks

    async def ingest_book_content(self, book_title: str, book_content: Dict[str, str], metadata: Dict[str, Any] = None) -> List[str]:
        """
        Ingest book content into the vector store
        book_content: Dictionary with section names as keys and content as values
        """
        if metadata is None:
            metadata = {}

        all_point_ids = []

        # Process each section of the book
        for section_name, content in book_content.items():
            # Chunk the content
            chunks = self.chunk_text(content)

            # Generate embeddings for all chunks at once for efficiency
            embeddings = self.embedding_service.generate_embeddings(chunks)

            # Prepare metadata for each chunk
            chunk_metadatas = []
            for i, chunk in enumerate(chunks):
                chunk_metadata = {
                    "book_title": book_title,
                    "section": section_name,
                    "chunk_index": i,
                    "total_chunks": len(chunks),
                    **metadata  # Include any additional metadata
                }
                chunk_metadatas.append(chunk_metadata)

            # Add all chunks to Qdrant
            point_ids = self.qdrant_service.batch_add_content(
                contents=chunks,
                metadatas=chunk_metadatas,
                embeddings=embeddings
            )

            all_point_ids.extend(point_ids)

        return all_point_ids

    def ingest_single_content(self, content: str, metadata: Dict[str, Any]) -> str:
        """
        Ingest a single piece of content with metadata
        """
        # Chunk the content
        chunks = self.chunk_text(content)

        # Process each chunk
        point_ids = []
        for chunk in chunks:
            # Generate embedding for the chunk
            embeddings = self.embedding_service.generate_embeddings([chunk])

            # Add to Qdrant
            point_id = self.qdrant_service.add_text_content(
                content=chunk,
                metadata=metadata,
                embedding=embeddings[0]
            )
            point_ids.append(point_id)

        # Return the first point ID (or handle multiple as needed)
        return point_ids[0] if point_ids else None