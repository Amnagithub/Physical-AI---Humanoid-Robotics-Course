from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List
from .base import BaseRepository
from ..models.database import Question


class QuestionRepository(BaseRepository):
    def __init__(self):
        super().__init__(Question)

    async def get_by_id(self, db: AsyncSession, question_id: str) -> Optional[Question]:
        """Get a question by its ID"""
        result = await db.execute(select(Question).where(Question.id == question_id))
        return result.scalar_one_or_none()

    async def get_by_session(self, db: AsyncSession, session_id: str) -> List[Question]:
        """Get all questions for a specific session"""
        result = await db.execute(select(Question).where(Question.session_id == session_id))
        return result.scalars().all()

    async def create_question(self, db: AsyncSession, session_id: str, content: str,
                            selected_text: Optional[str] = None, context_mode: str = 'full_book') -> Question:
        """Create a new question"""
        question_data = {
            'session_id': session_id,
            'content': content,
            'selected_text': selected_text,
            'context_mode': context_mode
        }
        return await self.create(db, question_data)