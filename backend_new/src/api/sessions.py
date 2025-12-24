from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
from uuid import UUID
from ..database import get_db
from ..repositories.session_repository import SessionRepository
from ..schemas.session import SessionCreateRequest, SessionResponse, SessionUpdateModeRequest, SessionModeResponse

router = APIRouter()


@router.post("/sessions", response_model=SessionResponse)
def create_session(session_create: SessionCreateRequest, db: Session = Depends(get_db)):
    """
    Create a new chat session
    """
    session_repo = SessionRepository(db)
    db_session = session_repo.create_session(mode=session_create.mode)
    return SessionResponse(
        id=db_session.id,
        created_at=db_session.created_at,
        mode=db_session.mode
    )


@router.get("/sessions/{session_id}", response_model=SessionResponse)
def get_session(session_id: UUID, db: Session = Depends(get_db)):
    """
    Get session details
    """
    session_repo = SessionRepository(db)
    db_session = session_repo.get_session(str(session_id))

    if not db_session:
        raise HTTPException(status_code=404, detail="Session not found")

    return SessionResponse(
        id=db_session.id,
        created_at=db_session.created_at,
        mode=db_session.mode
    )


@router.put("/sessions/{session_id}/mode", response_model=SessionModeResponse)
def update_session_mode(
    session_id: UUID,
    session_update: SessionUpdateModeRequest,
    db: Session = Depends(get_db)
):
    """
    Update session mode (full_book or selected_text)
    """
    session_repo = SessionRepository(db)
    db_session = session_repo.update_session_mode(str(session_id), session_update.mode)

    if not db_session:
        raise HTTPException(status_code=404, detail="Session not found")

    return SessionModeResponse(
        id=db_session.id,
        mode=db_session.mode
    )