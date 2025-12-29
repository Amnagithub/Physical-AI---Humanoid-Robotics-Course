from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
from uuid import UUID
import time
import logging
from ..database import get_db
from ..repositories.session_repository import SessionRepository
from ..repositories.question_repository import QuestionRepository
from ..repositories.answer_repository import AnswerRepository
from ..services.rag_service import RAGService
from ..schemas.question import QuestionCreateRequest, QuestionAnswerResponse, QuestionResponse, AnswerResponse

# Configure logging
logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/sessions/{session_id}/questions", response_model=QuestionAnswerResponse)
def submit_question(
    session_id: UUID,
    question_create: QuestionCreateRequest,
    db: Session = Depends(get_db)
):
    """
    Submit a question and get an answer
    """
    logger.info(f"📝 Received question for session {session_id}: {question_create.content[:100]}...")

    # Verify session exists
    session_repo = SessionRepository(db)
    session = session_repo.get_session(str(session_id))
    if not session:
        logger.warning(f"Session {session_id} not found")
        raise HTTPException(status_code=404, detail="Session not found")

    # Create question
    question_repo = QuestionRepository(db)
    db_question = question_repo.create_question(
        session_id=str(session_id),
        content=question_create.content,
        selected_text=question_create.selected_text,
        context_mode=session.mode
    )
    logger.info(f"Question created with ID: {db_question.id}")

    # Generate answer using RAG service
    rag_service = RAGService()
    logger.info("🔍 Starting RAG context retrieval...")

    # Measure response time for latency requirements
    start_time = time.time()
    answer_data = rag_service.answer_question(
        query=question_create.content,
        selected_text=question_create.selected_text
    )
    response_time = time.time() - start_time

    logger.info(f"✅ RAG response generated in {response_time:.2f}s")
    logger.info(f"   Answer length: {len(answer_data['content'])} chars")
    logger.info(f"   Sources: {answer_data['sources']}")
    logger.info(f"   Confidence score: {answer_data['confidence_score']:.2f}")

    # Check if response time exceeds the requirement (≤ 2s)
    if response_time > 2.0:
        logger.warning(f"⚠️ Response time {response_time:.2f}s exceeds 2s requirement")

    # Create answer
    answer_repo = AnswerRepository(db)
    db_answer = answer_repo.create_answer(
        question_id=str(db_question.id),
        content=answer_data["content"],
        sources=answer_data["sources"],
        confidence_score=answer_data["confidence_score"]
    )

    # Format response
    question_response = QuestionResponse(
        id=db_question.id,
        content=db_question.content,
        selected_text=db_question.selected_text
    )

    answer_response = AnswerResponse(
        id=db_answer.id,
        content=db_answer.content,
        sources=db_answer.sources,
        confidence_score=db_answer.confidence_score
    )

    return QuestionAnswerResponse(
        question=question_response,
        answer=answer_response
    )


@router.get("/sessions/{session_id}/history")
def get_conversation_history(session_id: UUID, db: Session = Depends(get_db)):
    """
    Get conversation history for a session
    """
    session_repo = SessionRepository(db)
    session = session_repo.get_session(str(session_id))
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    question_repo = QuestionRepository(db)
    questions = question_repo.get_questions_by_session(str(session_id))

    history = []
    for question in questions:
        answer_repo = AnswerRepository(db)
        answer = answer_repo.get_answers_by_question(str(question.id))

        history.append({
            "question": {
                "id": question.id,
                "content": question.content,
                "selected_text": question.selected_text,
                "created_at": question.created_at
            },
            "answer": {
                "id": answer.id if answer else None,
                "content": answer.content if answer else None,
                "sources": answer.sources if answer else [],
                "confidence_score": answer.confidence_score if answer else None,
                "created_at": answer.created_at if answer else None
            } if answer else None
        })

    return {"history": history}