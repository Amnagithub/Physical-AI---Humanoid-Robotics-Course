import os
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
import logging

logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self):
        # Get Qdrant configuration from environment
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Initialize Qdrant client
        if qdrant_api_key:
            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False
            )
        else:
            self.client = QdrantClient(host=os.getenv("QDRANT_HOST", "localhost"),
                                      port=int(os.getenv("QDRANT_PORT", "6333")))

        # Collection name for textbook content
        self.collection_name = "textbook_content"

        # Initialize the collection if it doesn't exist
        self._init_collection()

    def _init_collection(self):
        """Initialize the Qdrant collection for textbook content"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with vector configuration
                # Using OpenAI's text-embedding-ada-002 which produces 1536-dimensional vectors
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
                )

                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {e}")
            raise

    def add_textbook_content(self, content_id: str, content: str, section: str,
                           embedding: List[float], metadata: Optional[Dict[str, Any]] = None) -> bool:
        """Add textbook content to Qdrant with its embedding"""
        try:
            if metadata is None:
                metadata = {}

            # Add content to collection
            points = [PointStruct(
                id=content_id,
                vector=embedding,
                payload={
                    "content": content,
                    "section": section,
                    "metadata": metadata
                }
            )]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            return True
        except Exception as e:
            logger.error(f"Error adding textbook content to Qdrant: {e}")
            return False

    def search_content(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """Search for relevant content based on query embedding"""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                with_payload=True
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "section": result.payload.get("section", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score
                })

            return formatted_results
        except Exception as e:
            logger.error(f"Error searching content in Qdrant: {e}")
            return []

    def delete_content(self, content_id: str) -> bool:
        """Delete specific content from Qdrant"""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[content_id]
                )
            )
            return True
        except Exception as e:
            logger.error(f"Error deleting content from Qdrant: {e}")
            return False

    def get_content_by_id(self, content_id: str) -> Optional[Dict[str, Any]]:
        """Get specific content by ID"""
        try:
            results = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[content_id]
            )

            if results:
                result = results[0]
                return {
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "section": result.payload.get("section", ""),
                    "metadata": result.payload.get("metadata", {})
                }
            return None
        except Exception as e:
            logger.error(f"Error retrieving content from Qdrant: {e}")
            return None

    def get_all_content_ids(self) -> List[str]:
        """Get all content IDs in the collection"""
        try:
            records, _ = self.client.scroll(
                collection_name=self.collection_name,
                limit=10000,  # Adjust as needed
                with_payload=False,
                with_vectors=False
            )
            return [record.id for record in records]
        except Exception as e:
            logger.error(f"Error getting all content IDs from Qdrant: {e}")
            return []