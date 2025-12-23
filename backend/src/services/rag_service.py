import asyncio
import logging
from typing import List, Dict, Any, Optional, Tuple
from .qdrant_service import QdrantService
from .embedding_service import EmbeddingService
from .content_ingestion_service import ContentIngestionService

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self, qdrant_service: QdrantService, embedding_service: EmbeddingService):
        self.qdrant_service = qdrant_service
        self.embedding_service = embedding_service

    async def retrieve_content(self, query: str, limit: int = 5, mode: str = 'full_book') -> List[Dict[str, Any]]:
        """
        Retrieve relevant content based on the query.

        Args:
            query: The question or query text
            limit: Maximum number of results to return
            mode: 'full_book' or 'selected_text'

        Returns:
            List of relevant content with metadata
        """
        try:
            # Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(query)

            if not query_embedding:
                logger.error("Failed to generate embedding for query")
                return []

            # Search in Qdrant
            results = self.qdrant_service.search_content(
                query_embedding=query_embedding,
                limit=limit
            )

            return results

        except Exception as e:
            logger.error(f"Error retrieving content: {e}")
            return []

    async def retrieve_content_for_selected_text(self, selected_text: str, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve content based only on the selected text and query.
        This method will only return results that are related to the selected text.
        """
        try:
            # For selected text mode, we first embed the selected text to find similar content
            selected_text_embedding = await self.embedding_service.generate_embedding(selected_text)

            if not selected_text_embedding:
                logger.error("Failed to generate embedding for selected text")
                return []

            # Search in Qdrant for content similar to the selected text
            results = self.qdrant_service.search_content(
                query_embedding=selected_text_embedding,
                limit=limit
            )

            # Filter results to ensure they are relevant to both selected text and query
            # For now, we'll return the results from the selected text search
            # In a more sophisticated implementation, we might re-rank based on the query

            return results

        except Exception as e:
            logger.error(f"Error retrieving content for selected text: {e}")
            return []

    async def generate_response(self, query: str, selected_text: Optional[str] = None,
                              mode: str = 'full_book',
                              max_sources: int = 3) -> Tuple[str, List[Dict[str, Any]], float]:
        """
        Generate a response to the query using RAG.

        Args:
            query: The question to answer
            selected_text: Optional selected text for selected_text mode
            mode: 'full_book' or 'selected_text'
            max_sources: Maximum number of sources to reference

        Returns:
            Tuple of (response_text, sources, confidence_score)
        """
        try:
            # Retrieve relevant content
            if mode == 'selected_text' and selected_text:
                # Use selected text mode
                content_results = await self.retrieve_content_for_selected_text(
                    selected_text=selected_text,
                    query=query,
                    limit=max_sources
                )
            else:
                # Use full book mode
                content_results = await self.retrieve_content(
                    query=query,
                    limit=max_sources,
                    mode=mode
                )

            if not content_results:
                # No relevant content found
                return "This information is not covered in the book.", [], 0.0

            # Prepare context from retrieved content
            context_texts = [result['content'] for result in content_results if result['content'].strip()]

            if not context_texts:
                # Retrieved content was empty
                return "This information is not covered in the book.", [], 0.0

            # Combine context for the LLM
            context = "\n\n".join(context_texts)

            # Create a prompt for the LLM
            if mode == 'selected_text' and selected_text:
                prompt = f"""
                Based on the following selected text from the textbook, please answer the question:

                Selected Text: {selected_text}

                Additional Context: {context}

                Question: {query}

                Please provide an answer based only on the provided text. If the information is not available in the provided text, respond with "The selected text does not contain enough information to answer this question."

                Answer:
                """
            else:
                prompt = f"""
                Based on the following textbook content, please answer the question:

                Context: {context}

                Question: {query}

                Please provide an answer based only on the provided textbook content. If the information is not available in the provided content, respond with "This information is not covered in the book."

                Answer:
                """

            # Import the OpenAI service here to avoid circular dependencies
            from .openai_service import OpenAIService
            openai_service = OpenAIService()

            # Create a prompt for the LLM that includes proper source attribution
            if mode == 'selected_text' and selected_text:
                prompt = f"""
                Based on the following selected text from the textbook, please answer the question:

                Selected Text: {selected_text}

                Additional Context: {context}

                Question: {query}

                Please provide an answer based only on the provided text.
                Include specific citations to the textbook sections where the information comes from.
                If the information is not available in the provided text, respond with "The selected text does not contain enough information to answer this question."

                Answer:
                """
            else:
                prompt = f"""
                Based on the following textbook content, please answer the question:

                Context: {context}

                Question: {query}

                Please provide an answer based only on the provided textbook content.
                Include specific citations to the textbook sections where the information comes from.
                If the information is not available in the provided content, respond with "This information is not covered in the book."

                Answer:
                """

            # Generate response using OpenAI
            response = await openai_service.generate_response(
                prompt=prompt,
                system_message="You are an educational assistant that answers questions based only on provided textbook content. Always cite the specific sections where information comes from. If the requested information is not in the provided content, clearly state that it's not covered in the book."
            )

            if response is None:
                return "An error occurred while processing your request.", [], 0.0

            # Calculate a simple confidence score based on the number of sources and their scores
            if content_results:
                avg_score = sum(result['score'] for result in content_results) / len(content_results)
                confidence = min(avg_score, 1.0)  # Ensure confidence is between 0 and 1
            else:
                confidence = 0.0

            # Extract source information for attribution
            sources_for_attribution = []
            for result in content_results:
                source_info = {
                    'id': result.get('id'),
                    'section': result.get('section', 'Unknown Section'),
                    'score': result.get('score', 0.0),
                    'content_preview': result.get('content', '')[:200] + "..." if len(result.get('content', '')) > 200 else result.get('content', '')
                }
                sources_for_attribution.append(source_info)

            # Add source citations to the response if sources exist
            if sources_for_attribution:
                # Create a formatted list of sources
                source_list = "\n\nSources cited in this answer:\n"
                for i, source in enumerate(sources_for_attribution, 1):
                    source_list += f"{i}. {source.get('section', 'Unknown Section')}\n"

                # Append the sources to the response
                response_with_citations = f"{response}\n\n{source_list.strip()}"
            else:
                response_with_citations = response

            return response_with_citations, sources_for_attribution, confidence

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            return "An error occurred while processing your request.", [], 0.0

    async def check_content_coverage(self, query: str) -> bool:
        """
        Check if the query content is covered in the textbook.

        Args:
            query: The question to check

        Returns:
            True if content is likely covered, False otherwise
        """
        try:
            # Retrieve content for the query
            results = await self.retrieve_content(query, limit=1)
            return len(results) > 0
        except Exception as e:
            logger.error(f"Error checking content coverage: {e}")
            return False