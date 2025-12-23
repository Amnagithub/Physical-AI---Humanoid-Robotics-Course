from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List
from .base import BaseRepository
from ..models.database import TextbookContent


class TextbookRepository(BaseRepository):
    def __init__(self):
        super().__init__(TextbookContent)

    async def get_by_id(self, db: AsyncSession, content_id: str) -> Optional[TextbookContent]:
        """Get textbook content by its ID"""
        result = await db.execute(select(TextbookContent).where(TextbookContent.id == content_id))
        return result.scalar_one_or_none()

    async def get_by_section(self, db: AsyncSession, section: str) -> Optional[TextbookContent]:
        """Get textbook content by section"""
        result = await db.execute(select(TextbookContent).where(TextbookContent.section == section))
        return result.scalar_one_or_none()

    async def search_by_content(self, db: AsyncSession, search_text: str) -> List[TextbookContent]:
        """Search textbook content by text"""
        result = await db.execute(
            select(TextbookContent).where(TextbookContent.content.contains(search_text))
        )
        return result.scalars().all()

    async def create_content(self, db: AsyncSession, section: str, content: str, metadata: dict = None) -> TextbookContent:
        """Create new textbook content"""
        if metadata is None:
            metadata = {}

        content_data = {
            'section': section,
            'content': content,
            'metadata': metadata
        }
        return await self.create(db, content_data)