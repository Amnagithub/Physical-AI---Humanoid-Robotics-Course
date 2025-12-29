from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from uuid import uuid4
import numpy as np
from ..config import settings


class QdrantService:
    def __init__(self):
        # Initialize Qdrant client
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Ensure the collection exists with proper configuration
        """
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.collection_name not in collection_names:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),  # Cohere embeddings are 1024-dim
            )

    def store_embeddings(self, texts: List[str], metadata_list: List[Dict[str, Any]]) -> List[str]:
        """
        Store text embeddings in Qdrant
        """
        # Generate IDs for the points
        ids = [str(uuid4()) for _ in texts]

        # In a real implementation, we'd call the embedding service here
        # For now, we'll assume embeddings are already generated
        # This method would be called from an ingestion service

        return ids

    def search_similar(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar content based on query embedding
        """
        search_results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        results = []
        for hit in search_results.points:
            results.append({
                'id': hit.id,
                'content': hit.payload.get('content', ''),
                'metadata': hit.payload.get('metadata', {}),
                'score': hit.score
            })

        return results

    def add_text_content(self, content: str, metadata: Dict[str, Any], embedding: List[float]) -> str:
        """
        Add a single text content with its embedding to the collection
        """
        point_id = str(uuid4())

        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        "metadata": metadata
                    }
                )
            ]
        )

        return point_id

    def batch_add_content(self, contents: List[str], metadatas: List[Dict[str, Any]], embeddings: List[List[float]]) -> List[str]:
        """
        Add multiple text contents with their embeddings to the collection
        """
        ids = [str(uuid4()) for _ in contents]

        points = [
            models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": content,
                    "metadata": metadata
                }
            )
            for point_id, content, metadata, embedding in zip(ids, contents, metadatas, embeddings)
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return ids

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection
        """
        try:
            collection_info = self.client.get_collection(collection_name=self.collection_name)
            points_count = self.client.count(
                collection_name=self.collection_name,
                exact=True
            )
            return {
                "collection_name": self.collection_name,
                "status": collection_info.status,
                "vectors_count": points_count.count,
                "config": {
                    "vector_size": collection_info.config.params.size,
                    "distance": str(collection_info.config.params.distance)
                }
            }
        except Exception as e:
            return {
                "error": str(e),
                "collection_name": self.collection_name
            }