from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
from pydantic import BaseModel

from ..database import get_db
from ..models.database import StudentSession
from ..repositories.session_repository import SessionRepository
from ..schemas.session import SessionCreate, SessionResponse

class UpdateSessionMode(BaseModel):
    mode: str

router = APIRouter(prefix="/sessions", tags=["sessions"])

@router.post("/", response_model=SessionResponse)
async def create_session(
    session_data: SessionCreate = None,
    db: AsyncSession = Depends(get_db)
):
    """Create a new chat session"""
    if session_data is None:
        # Default to full_book mode if no data provided
        session_data = SessionCreate(mode="full_book")

    # Use repository to create session
    session_repo = SessionRepository()
    session = await session_repo.create_session(db, session_data.mode)

    return SessionResponse(
        id=session.id,
        created_at=session.created_at.isoformat(),
        mode=session.mode
    )

@router.get("/{session_id}", response_model=SessionResponse)
async def get_session(
    session_id: str,
    db: AsyncSession = Depends(get_db)
):
    """Get session details"""
    session_repo = SessionRepository()
    session = await session_repo.get_by_id(db, session_id)

    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Session not found"
        )

    return SessionResponse(
        id=session.id,
        created_at=session.created_at.isoformat(),
        mode=session.mode,
        last_activity=session.last_activity.isoformat() if session.last_activity else None
    )

@router.put("/{session_id}/mode", response_model=SessionResponse)
async def update_session_mode(
    session_id: str,
    mode_data: UpdateSessionMode,
    db: AsyncSession = Depends(get_db)
):
    """Update session mode"""
    mode = mode_data.mode
    if mode not in ["full_book", "selected_text"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Mode must be either 'full_book' or 'selected_text'"
        )

    session_repo = SessionRepository()
    updated_session = await session_repo.update_session_mode(db, session_id, mode)

    if not updated_session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Session not found"
        )

    return SessionResponse(
        id=updated_session.id,
        created_at=updated_session.created_at.isoformat(),
        mode=updated_session.mode,
        last_activity=updated_session.last_activity.isoformat() if updated_session.last_activity else None
    )