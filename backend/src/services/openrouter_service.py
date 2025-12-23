import os
import asyncio
import logging
from typing import List, Dict, Any, Optional
import aiohttp
import json
import tiktoken

logger = logging.getLogger(__name__)

class OpenRouterService:
    def __init__(self):
        # Initialize OpenRouter client
        api_key = os.getenv("OPENROUTER_API_KEY")
        if not api_key:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")

        self.api_key = api_key
        self.base_url = "https://openrouter.ai/api/v1"
        self.model = os.getenv("OPENROUTER_MODEL", "qwen/qwen-2-72b-instruct")
        self.embedding_model = os.getenv("OPENROUTER_EMBEDDING_MODEL", "nomic-ai/nomic-embed-text-v1.5")

        # Initialize tokenizer for the model
        try:
            # Using gpt-4 tokenizer as a fallback since Qwen models don't have specific tiktoken encodings
            self.tokenizer = tiktoken.encoding_for_model("gpt-4")
        except KeyError:
            # Fallback to cl100k_base
            self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def _truncate_text(self, text: str, max_tokens: int = 32768) -> str:
        """Truncate text to fit within token limit for Qwen models"""
        tokens = self.tokenizer.encode(text)
        if len(tokens) <= max_tokens:
            return text

        # Truncate tokens and decode back to text
        truncated_tokens = tokens[:max_tokens]
        truncated_text = self.tokenizer.decode(truncated_tokens)
        return truncated_text

    async def generate_response(self, prompt: str, system_message: str = None,
                              max_tokens: int = 1000, temperature: float = 0.3) -> Optional[str]:
        """
        Generate a response using OpenRouter API with Qwen model.
        """
        try:
            messages = []

            # Add system message if provided
            if system_message:
                messages.append({
                    "role": "system",
                    "content": system_message
                })

            # Add user message (the prompt)
            messages.append({
                "role": "user",
                "content": prompt
            })

            # Prepare the API call parameters
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }

            data = {
                "model": self.model,
                "messages": messages,
                "max_tokens": max_tokens,
                "temperature": temperature,
                "timeout": 30
            }

            # Call OpenRouter API
            async with aiohttp.ClientSession() as session:
                async with session.post(f"{self.base_url}/chat/completions",
                                      headers=headers,
                                      json=data) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"OpenRouter API error: {response.status} - {error_text}")
                        return None

                    response_json = await response.json()

                    # Extract the response text
                    generated_text = response_json["choices"][0]["message"]["content"].strip()
                    return generated_text

        except aiohttp.ClientError as e:
            logger.error(f"OpenRouter API error during response generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during response generation: {e}")
            return None

    async def generate_embedding(self, text: str) -> Optional[List[float]]:
        """Generate embedding for the given text using OpenRouter API with Qwen embedding model"""
        try:
            # Truncate text if it exceeds the model's token limit
            truncated_text = self._truncate_text(text)

            if not truncated_text.strip():
                logger.warning("Empty text provided for embedding generation")
                return None

            # Prepare the API call parameters
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }

            data = {
                "model": self.embedding_model,
                "input": truncated_text
            }

            # Call OpenRouter API for embeddings
            async with aiohttp.ClientSession() as session:
                async with session.post(f"{self.base_url}/embeddings",
                                      headers=headers,
                                      json=data) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"OpenRouter embedding API error: {response.status} - {error_text}")
                        return None

                    response_json = await response.json()

                    # Extract embedding from response
                    embedding = response_json["data"][0]["embedding"]
                    return embedding

        except aiohttp.ClientError as e:
            logger.error(f"OpenRouter API error during embedding generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during embedding generation: {e}")
            return None

    async def generate_embeddings_batch(self, texts: List[str]) -> Optional[List[List[float]]]:
        """Generate embeddings for a batch of texts"""
        try:
            # Truncate each text if necessary
            truncated_texts = [self._truncate_text(text) for text in texts if text.strip()]

            if not truncated_texts:
                logger.warning("No valid texts provided for batch embedding generation")
                return None

            # Prepare the API call parameters
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }

            data = {
                "model": self.embedding_model,
                "input": truncated_texts
            }

            # Call OpenRouter API for embeddings
            async with aiohttp.ClientSession() as session:
                async with session.post(f"{self.base_url}/embeddings",
                                      headers=headers,
                                      json=data) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"OpenRouter embedding API error: {response.status} - {error_text}")
                        return None

                    response_json = await response.json()

                    # Extract embeddings from response
                    embeddings = [item["embedding"] for item in response_json["data"]]
                    return embeddings

        except aiohttp.ClientError as e:
            logger.error(f"OpenRouter API error during batch embedding generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during batch embedding generation: {e}")
            return None

    def count_tokens(self, text: str) -> int:
        """Count the number of tokens in the given text"""
        try:
            tokens = self.tokenizer.encode(text)
            return len(tokens)
        except Exception as e:
            logger.error(f"Error counting tokens: {e}")
            return 0

    async def validate_text_for_embedding(self, text: str, max_tokens: int = 32768) -> bool:
        """Validate if text is suitable for embedding generation"""
        if not text or not text.strip():
            return False

        token_count = self.count_tokens(text)
        return token_count <= max_tokens