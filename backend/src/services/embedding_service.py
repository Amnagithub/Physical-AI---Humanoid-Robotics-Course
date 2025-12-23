import os
import asyncio
import logging
from typing import List, Optional
import openai
from openai import AsyncOpenAI
import tiktoken
from ..config import settings

logger = logging.getLogger(__name__)

class EmbeddingService:
    def __init__(self):
        self.provider = settings.llm_provider.lower()

        if self.provider == "openrouter":
            # Use OpenRouter for embeddings
            from .openrouter_service import OpenRouterService
            api_key = os.getenv("OPENROUTER_API_KEY")
            if not api_key:
                raise ValueError("OPENROUTER_API_KEY environment variable is required")
            self.openrouter_service = OpenRouterService()
        else:
            # Initialize OpenAI client
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY environment variable is required")

            self.client = AsyncOpenAI(api_key=api_key)
            self.model = os.getenv("EMBEDDING_MODEL", "text-embedding-ada-002")

            # Initialize tokenizer for the embedding model
            try:
                self.tokenizer = tiktoken.encoding_for_model(self.model)
            except KeyError:
                # Fallback to cl100k_base which is used by text-embedding-ada-002
                self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def _truncate_text(self, text: str, max_tokens: int = 8191) -> str:
        """Truncate text to fit within token limit"""
        if self.provider == "openrouter":
            # For OpenRouter, use the service's tokenizer
            tokens = self.openrouter_service.tokenizer.encode(text)
            if len(tokens) <= max_tokens:
                return text

            # Truncate tokens and decode back to text
            truncated_tokens = tokens[:max_tokens]
            truncated_text = self.openrouter_service.tokenizer.decode(truncated_tokens)
            return truncated_text
        else:
            tokens = self.tokenizer.encode(text)
            if len(tokens) <= max_tokens:
                return text

            # Truncate tokens and decode back to text
            truncated_tokens = tokens[:max_tokens]
            truncated_text = self.tokenizer.decode(truncated_tokens)
            return truncated_text

    async def generate_embedding(self, text: str) -> Optional[List[float]]:
        """Generate embedding for the given text using selected provider API"""
        try:
            if self.provider == "openrouter":
                # Use OpenRouter for embedding generation
                return await self.openrouter_service.generate_embedding(text)
            else:
                # Truncate text if it exceeds the model's token limit
                truncated_text = self._truncate_text(text)

                if not truncated_text.strip():
                    logger.warning("Empty text provided for embedding generation")
                    return None

                # Generate embedding using OpenAI API
                response = await self.client.embeddings.create(
                    input=truncated_text,
                    model=self.model
                )

                # Extract embedding from response
                embedding = response.data[0].embedding
                return embedding

        except openai.APIError as e:
            logger.error(f"OpenAI API error during embedding generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during embedding generation: {e}")
            return None

    async def generate_embeddings_batch(self, texts: List[str]) -> Optional[List[List[float]]]:
        """Generate embeddings for a batch of texts"""
        try:
            if self.provider == "openrouter":
                # Use OpenRouter for batch embedding generation
                return await self.openrouter_service.generate_embeddings_batch(texts)
            else:
                # Truncate each text if necessary
                truncated_texts = [self._truncate_text(text) for text in texts if text.strip()]

                if not truncated_texts:
                    logger.warning("No valid texts provided for batch embedding generation")
                    return None

                # Generate embeddings using OpenAI API
                response = await self.client.embeddings.create(
                    input=truncated_texts,
                    model=self.model
                )

                # Extract embeddings from response
                embeddings = [item.embedding for item in response.data]
                return embeddings

        except openai.APIError as e:
            logger.error(f"OpenAI API error during batch embedding generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during batch embedding generation: {e}")
            return None

    def count_tokens(self, text: str) -> int:
        """Count the number of tokens in the given text"""
        try:
            if self.provider == "openrouter":
                tokens = self.openrouter_service.tokenizer.encode(text)
            else:
                tokens = self.tokenizer.encode(text)
            return len(tokens)
        except Exception as e:
            logger.error(f"Error counting tokens: {e}")
            return 0

    async def validate_text_for_embedding(self, text: str, max_tokens: int = 8191) -> bool:
        """Validate if text is suitable for embedding generation"""
        if not text or not text.strip():
            return False

        token_count = self.count_tokens(text)
        return token_count <= max_tokens