from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List
import uuid

from ..database import get_db
from ..repositories.session_repository import SessionRepository
from ..repositories.question_repository import QuestionRepository
from ..repositories.answer_repository import AnswerRepository
from ..schemas.question import QuestionCreate, QuestionResponse, AnswerResponse
from ..services.rag_service import RAGService
from ..services.qdrant_service import QdrantService
from ..services.embedding_service import EmbeddingService
from ..config import settings

router = APIRouter(prefix="/sessions/{session_id}/questions", tags=["questions"])

# Initialize services (in a real app, these would be dependency injected)
qdrant_service = QdrantService()
embedding_service = EmbeddingService()
rag_service = RAGService(qdrant_service, embedding_service)

@router.post("/", response_model=AnswerResponse)
async def submit_question(
    session_id: str,
    question_data: QuestionCreate,
    db: AsyncSession = Depends(get_db)
):
    """Submit a question and get an answer"""
    # Validate input
    if not question_data.content or not question_data.content.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Question content cannot be empty"
        )

    # Check if session exists
    session_repo = SessionRepository()
    session = await session_repo.get_by_id(db, session_id)

    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Session not found"
        )

    # Validate mode if provided
    if question_data.mode and question_data.mode not in ["full_book", "selected_text"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Mode must be either 'full_book' or 'selected_text'"
        )

    # Validate selected_text if in selected_text mode
    if (question_data.mode == "selected_text" or session.mode == "selected_text") and question_data.selected_text:
        if not question_data.selected_text.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Selected text cannot be empty in selected_text mode"
            )

    # Create the question
    question_repo = QuestionRepository()
    question = await question_repo.create_question(
        db,
        session_id=session_id,
        content=question_data.content.strip(),
        selected_text=question_data.selected_text.strip() if question_data.selected_text else None,
        context_mode=question_data.mode or session.mode
    )

    # Get answer using RAG service
    try:
        # Determine mode to use
        mode = question_data.mode or session.mode

        answer_text, sources, confidence = await rag_service.generate_response(
            query=question_data.content,
            selected_text=question_data.selected_text,
            mode=mode
        )

        # Format the response with proper source citations
        formatted_sources = [source.get('section', '') for source in sources]

        # Create the answer
        answer_repo = AnswerRepository()
        answer = await answer_repo.create_answer(
            db,
            question_id=question.id,
            content=answer_text,
            sources=formatted_sources,
            confidence_score=confidence
        )

        # Update session last activity
        await session_repo.update_last_activity(db, session_id)

        return AnswerResponse(
            id=answer.id,
            question_id=answer.question_id,
            content=answer.content,
            sources=answer.sources,
            created_at=answer.created_at.isoformat(),
            confidence_score=answer.confidence_score
        )

    except Exception as e:
        import traceback
        logger.error(f"Error processing question: {e}\n{traceback.format_exc()}")

        # Try to create an error response in the database
        try:
            answer_repo = AnswerRepository()
            answer = await answer_repo.create_answer(
                db,
                question_id=question.id,
                content="An error occurred while processing your question.",
                sources=[],
                confidence_score=0.0
            )
        except Exception as db_error:
            logger.error(f"Error saving error response to database: {db_error}")
            pass  # Continue with raising the HTTP exception

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error processing question"
        )


@router.get("/", response_model=List[QuestionResponse])
async def get_session_questions(
    session_id: str,
    skip: int = 0,
    limit: int = 100,
    db: AsyncSession = Depends(get_db)
):
    """Get all questions for a session"""
    # Check if session exists
    session_repo = SessionRepository()
    session = await session_repo.get_by_id(db, session_id)

    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Session not found"
        )

    # Get questions for the session
    question_repo = QuestionRepository()
    questions = await question_repo.get_by_session(db, session_id)

    return [
        QuestionResponse(
            id=q.id,
            session_id=q.session_id,
            content=q.content,
            created_at=q.created_at.isoformat(),
            selected_text=q.selected_text,
            context_mode=q.context_mode
        )
        for q in questions
    ]