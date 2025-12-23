from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional
from .base import BaseRepository
from ..models.database import Answer


class AnswerRepository(BaseRepository):
    def __init__(self):
        super().__init__(Answer)

    async def get_by_id(self, db: AsyncSession, answer_id: str) -> Optional[Answer]:
        """Get an answer by its ID"""
        result = await db.execute(select(Answer).where(Answer.id == answer_id))
        return result.scalar_one_or_none()

    async def get_by_question_id(self, db: AsyncSession, question_id: str) -> Optional[Answer]:
        """Get an answer by its associated question ID"""
        result = await db.execute(select(Answer).where(Answer.question_id == question_id))
        return result.scalar_one_or_none()

    async def create_answer(self, db: AsyncSession, question_id: str, content: str,
                           sources: list = None, confidence_score: float = 0.0) -> Answer:
        """Create a new answer"""
        if sources is None:
            sources = []

        answer_data = {
            'question_id': question_id,
            'content': content,
            'sources': sources,
            'confidence_score': confidence_score
        }
        return await self.create(db, answer_data)

    async def update_answer(self, db: AsyncSession, answer_id: str, content: str = None,
                           sources: list = None, confidence_score: float = None) -> Optional[Answer]:
        """Update an existing answer"""
        answer = await self.get_by_id(db, answer_id)
        if answer:
            update_data = {}
            if content is not None:
                update_data['content'] = content
            if sources is not None:
                update_data['sources'] = sources
            if confidence_score is not None:
                update_data['confidence_score'] = confidence_score

            for field, value in update_data.items():
                setattr(answer, field, value)

            await db.commit()
            await db.refresh(answer)
        return answer