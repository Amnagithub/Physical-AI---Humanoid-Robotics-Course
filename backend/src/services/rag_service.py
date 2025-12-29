import cohere
from typing import List, Dict, Any, Optional
from ..config import settings
from ..services.embedding_service import EmbeddingService
from ..services.qdrant_service import QdrantService


class RAGService:
    def __init__(self):
        self.cohere_client = cohere.ClientV2(api_key=settings.COHERE_API_KEY)
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()

    def retrieve_context(self, query: str, top_k: int = 5, min_score: float = 0.85) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the vector store based on the query
        """
        # Generate embedding for the query
        query_embedding = self.embedding_service.generate_query_embedding(query)

        # Search for similar content in Qdrant
        search_results = self.qdrant_service.search_similar(
            query_embedding=query_embedding,
            top_k=top_k
        )

        # Filter results by minimum score
        filtered_results = [
            result for result in search_results
            if result['score'] >= min_score
        ]

        return filtered_results

    def generate_response(self, query: str, context: List[Dict[str, Any]], selected_text: Optional[str] = None) -> Dict[str, Any]:
        """
        Generate a response using Cohere based on the query and context
        """
        # Determine which context to use
        if selected_text:
            # Use selected text only mode
            context_text = selected_text
            sources = ["selected_text"]
        else:
            # Use retrieved context from vector store
            context_parts = [item['content'] for item in context]
            context_text = "\n\n".join(context_parts)
            sources = [item['id'] for item in context]

        # Prepare the prompt for Cohere
        if context_text.strip():
            prompt = f"""
            Answer the question based on the provided context. If the context does not contain sufficient information to answer the question, respond with exactly: "The provided text does not contain sufficient information to answer this question."

            Context:
            {context_text}

            Question: {query}

            Answer:"""
        else:
            # If no context is provided, we should refuse to answer
            return {
                "content": "The provided text does not contain sufficient information to answer this question.",
                "sources": [],
                "confidence_score": 0.0
            }

        # Generate response using Cohere Chat API v2
        system_message = "You are a helpful assistant that answers questions based on the provided context. If the context does not contain sufficient information to answer the question, respond with exactly: 'The provided text does not contain sufficient information to answer this question.'"

        user_message = f"Context:\n{context_text}\n\nQuestion: {query}"

        response = self.cohere_client.chat(
            model="command-a-03-2025",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_message}
            ],
            temperature=0.3,
        )

        # Extract the generated text
        generated_text = response.message.content[0].text.strip()

        # Check if the response contains the refusal message
        if "The provided text does not contain sufficient information to answer this question." in generated_text:
            return {
                "content": "The provided text does not contain sufficient information to answer this question.",
                "sources": [],
                "confidence_score": 0.0
            }

        # Calculate a basic confidence score based on context match
        confidence_score = self._calculate_confidence_score(query, generated_text, context)

        return {
            "content": generated_text,
            "sources": sources,
            "confidence_score": confidence_score
        }

    def _calculate_confidence_score(self, query: str, response: str, context: List[Dict[str, Any]]) -> float:
        """
        Calculate a basic confidence score based on similarity between query, context, and response
        """
        if not context:
            return 0.0

        # Simple approach: calculate average similarity score from the retrieved context
        if context:
            avg_score = sum(item['score'] for item in context) / len(context)
            return min(avg_score, 1.0)  # Ensure it doesn't exceed 1.0

        return 0.5  # Default score if no context was used

    def answer_question(self, query: str, selected_text: Optional[str] = None) -> Dict[str, Any]:
        """
        Main method to answer a question using RAG
        """
        # If selected text is provided, use it directly (Selected Text Only mode)
        if selected_text:
            return self.generate_response(query, context=[], selected_text=selected_text)

        # Otherwise, retrieve context from the vector store (Full Book RAG mode)
        context = self.retrieve_context(
            query=query,
            top_k=settings.TOP_K,
            min_score=settings.CONFIDENCE_THRESHOLD
        )

        # If no context is found with sufficient score, return refusal message
        if not context:
            return {
                "content": "The provided text does not contain sufficient information to answer this question.",
                "sources": [],
                "confidence_score": 0.0
            }

        # Generate response using the retrieved context
        return self.generate_response(query, context)