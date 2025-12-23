from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional
from .base import BaseRepository
from ..models.database import StudentSession


class SessionRepository(BaseRepository):
    def __init__(self):
        super().__init__(StudentSession)

    async def get_by_id(self, db: AsyncSession, session_id: str) -> Optional[StudentSession]:
        """Get a session by its ID"""
        result = await db.execute(select(StudentSession).where(StudentSession.id == session_id))
        return result.scalar_one_or_none()

    async def create_session(self, db: AsyncSession, mode: str = 'full_book') -> StudentSession:
        """Create a new student session"""
        session_data = {
            'mode': mode
        }
        return await self.create(db, session_data)

    async def update_session_mode(self, db: AsyncSession, session_id: str, mode: str) -> Optional[StudentSession]:
        """Update the mode of a session"""
        session = await self.get_by_id(db, session_id)
        if session:
            session.mode = mode
            db.add(session)
            await db.commit()
            await db.refresh(session)
        return session

    async def update_last_activity(self, db: AsyncSession, session_id: str):
        """Update the last activity timestamp for a session"""
        session = await self.get_by_id(db, session_id)
        if session:
            from datetime import datetime
            session.last_activity = datetime.utcnow()
            db.add(session)
            await db.commit()
            await db.refresh(session)
        return session