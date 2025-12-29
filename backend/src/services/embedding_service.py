import cohere
from typing import List, Dict, Any
from ..config import settings


class EmbeddingService:
    def __init__(self):
        self.client = cohere.ClientV2(api_key=settings.COHERE_API_KEY)

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        """
        response = self.client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document",
            embedding_types=["float"]
        )
        return response.embeddings.float_

    def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for a query using Cohere
        """
        response = self.client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query",
            embedding_types=["float"]
        )
        return response.embeddings.float_[0]