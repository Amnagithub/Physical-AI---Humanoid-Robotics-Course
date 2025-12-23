import os
import asyncio
import logging
from typing import List, Dict, Any, Optional
import openai
from openai import AsyncOpenAI
import json
from ..config import settings

logger = logging.getLogger(__name__)

class OpenAIService:
    def __init__(self):
        self.provider = settings.llm_provider.lower()

        if self.provider == "openrouter":
            # Use OpenRouter
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
            self.model = os.getenv("OPENAI_MODEL", "gpt-4-turbo-preview")

    async def generate_response(self, prompt: str, system_message: str = None,
                              max_tokens: int = 1000, temperature: float = 0.3) -> Optional[str]:
        """
        Generate a response using selected provider API.

        Args:
            prompt: The user's query or prompt
            system_message: Optional system message to guide the model's behavior
            max_tokens: Maximum number of tokens to generate
            temperature: Controls randomness (0.0-2.0)

        Returns:
            Generated response text or None if error
        """
        try:
            if self.provider == "openrouter":
                # Use OpenRouter for response generation
                return await self.openrouter_service.generate_response(
                    prompt, system_message, max_tokens, temperature
                )
            else:
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

                # Call OpenAI API
                response = await self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    max_tokens=max_tokens,
                    temperature=temperature,
                    timeout=30  # 30 second timeout
                )

                # Extract the response text
                generated_text = response.choices[0].message.content.strip()
                return generated_text

        except openai.APIError as e:
            logger.error(f"OpenAI API error during response generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during response generation: {e}")
            return None

    async def generate_structured_response(self, prompt: str, system_message: str = None,
                                         response_format: Dict[str, Any] = None,
                                         max_tokens: int = 1000, temperature: float = 0.3) -> Optional[Dict[str, Any]]:
        """
        Generate a structured response using selected provider API.

        Args:
            prompt: The user's query or prompt
            system_message: Optional system message to guide the model's behavior
            response_format: Expected response format (for structured output)
            max_tokens: Maximum number of tokens to generate
            temperature: Controls randomness (0.0-2.0)

        Returns:
            Generated response as dictionary or None if error
        """
        try:
            if self.provider == "openrouter":
                # For OpenRouter, generate a regular response first, then try to parse it as structured
                response_text = await self.openrouter_service.generate_response(
                    prompt, system_message, max_tokens, temperature
                )

                if response_text:
                    # Try to parse as JSON if expected
                    if response_format:
                        try:
                            # Remove any markdown code block markers if present
                            if response_text.startswith('```json'):
                                response_text = response_text[7:]  # Remove ```json
                            if response_text.endswith('```'):
                                response_text = response_text[:-3]  # Remove ```

                            # Parse the JSON
                            structured_response = json.loads(response_text)
                            return structured_response
                        except json.JSONDecodeError:
                            logger.warning("Could not parse response as JSON, returning as text")
                            return {"response": response_text}

                    return {"response": response_text}
                else:
                    return None
            else:
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
                params = {
                    "model": self.model,
                    "messages": messages,
                    "max_tokens": max_tokens,
                    "temperature": temperature,
                    "timeout": 30  # 30 second timeout
                }

                # Add response format if specified
                if response_format:
                    # Note: OpenAI's structured output feature may require specific model support
                    # For now, we'll generate the response and attempt to parse it as JSON
                    pass

                # Call OpenAI API
                response = await self.client.chat.completions.create(**params)

                # Extract the response text
                generated_text = response.choices[0].message.content.strip()

                # Try to parse as JSON if expected
                if response_format:
                    try:
                        # Remove any markdown code block markers if present
                        if generated_text.startswith('```json'):
                            generated_text = generated_text[7:]  # Remove ```json
                        if generated_text.endswith('```'):
                            generated_text = generated_text[:-3]  # Remove ```

                        # Parse the JSON
                        structured_response = json.loads(generated_text)
                        return structured_response
                    except json.JSONDecodeError:
                        logger.warning("Could not parse response as JSON, returning as text")
                        return {"response": generated_text}

                return {"response": generated_text}

        except openai.APIError as e:
            logger.error(f"OpenAI API error during structured response generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during structured response generation: {e}")
            return None

    async def validate_response_quality(self, query: str, response: str, context: str) -> Dict[str, Any]:
        """
        Validate the quality of a response by checking if it's relevant to the query and context.

        Args:
            query: Original query
            response: Generated response
            context: Context used to generate the response

        Returns:
            Dictionary with validation results
        """
        try:
            validation_prompt = f"""
            Please evaluate the following response based on the query and context provided.

            Query: {query}

            Context: {context}

            Response: {response}

            Evaluate the response on the following criteria:
            1. Relevance: How well does the response address the query?
            2. Faithfulness: Does the response align with the provided context?
            3. Accuracy: Is the information in the response factually correct based on the context?
            4. Completeness: Does the response adequately address the query?

            Please provide scores from 1-5 for each criterion (1 = poor, 5 = excellent)
            and a brief explanation for each score.

            Return your response in the following JSON format:
            {{
                "relevance_score": 0,
                "faithfulness_score": 0,
                "accuracy_score": 0,
                "completeness_score": 0,
                "explanation": {{
                    "relevance": "",
                    "faithfulness": "",
                    "accuracy": "",
                    "completeness": ""
                }},
                "overall_quality": ""
            }}
            """

            validation_result = await self.generate_structured_response(
                prompt=validation_prompt,
                system_message="You are an expert evaluator of AI-generated responses. Evaluate responses based on their relevance, faithfulness to context, accuracy, and completeness."
            )

            return validation_result

        except Exception as e:
            logger.error(f"Error validating response quality: {e}")
            return {
                "relevance_score": 0,
                "faithfulness_score": 0,
                "accuracy_score": 0,
                "completeness_score": 0,
                "explanation": {
                    "relevance": "Error during validation",
                    "faithfulness": "Error during validation",
                    "accuracy": "Error during validation",
                    "completeness": "Error during validation"
                },
                "overall_quality": "Validation failed"
            }

    async def extract_sources(self, response: str, context_chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Extract source references from the response based on context chunks.

        Args:
            response: Generated response that may reference context
            context_chunks: List of context chunks with metadata

        Returns:
            List of source references
        """
        try:
            # For now, return all context chunks as potential sources
            # In a more sophisticated implementation, we would analyze the response
            # to identify which specific parts came from which sources
            return context_chunks

        except Exception as e:
            logger.error(f"Error extracting sources: {e}")
            return []