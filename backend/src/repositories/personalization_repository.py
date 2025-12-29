"""
Repository for personalized content CRUD operations
Feature: 002-user-auth-personalization
"""

from typing import Optional, List
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
from sqlalchemy import and_

from ..models.personalized_content import PersonalizedContent


class PersonalizationRepository:
    """Repository for managing personalized content cache entries"""

    def __init__(self, db: Session):
        self.db = db

    def get_cached_content(
        self,
        user_id: str,
        chapter_path: str,
        content_type: str,
        language: str = "en"
    ) -> Optional[PersonalizedContent]:
        """
        Get cached personalized content if it exists and is not expired.

        Args:
            user_id: The user's ID
            chapter_path: The chapter path (e.g., /docs/modules/ros2/chapter1)
            content_type: Type of content (personalized, translated, both)
            language: Target language code

        Returns:
            PersonalizedContent if found and valid, None otherwise
        """
        return self.db.query(PersonalizedContent).filter(
            and_(
                PersonalizedContent.user_id == user_id,
                PersonalizedContent.chapter_path == chapter_path,
                PersonalizedContent.content_type == content_type,
                PersonalizedContent.language == language,
                PersonalizedContent.expires_at > datetime.utcnow()
            )
        ).first()

    def store_cached_content(
        self,
        user_id: str,
        chapter_path: str,
        content_type: str,
        original_hash: str,
        generated_content: str,
        language: str = "en",
        expires_in_days: int = 7
    ) -> PersonalizedContent:
        """
        Store or update cached personalized content.

        Args:
            user_id: The user's ID
            chapter_path: The chapter path
            content_type: Type of content transformation
            original_hash: MD5 hash of original content
            generated_content: The generated content
            language: Target language code
            expires_in_days: Cache expiration in days

        Returns:
            The created/updated PersonalizedContent entry
        """
        now = datetime.utcnow()
        expires_at = now + timedelta(days=expires_in_days)

        # Check if entry exists
        existing = self.db.query(PersonalizedContent).filter(
            and_(
                PersonalizedContent.user_id == user_id,
                PersonalizedContent.chapter_path == chapter_path,
                PersonalizedContent.content_type == content_type,
                PersonalizedContent.language == language
            )
        ).first()

        if existing:
            # Update existing entry
            existing.original_hash = original_hash
            existing.generated_content = generated_content
            existing.created_at = now
            existing.expires_at = expires_at
            self.db.commit()
            self.db.refresh(existing)
            return existing
        else:
            # Create new entry
            new_entry = PersonalizedContent(
                user_id=user_id,
                chapter_path=chapter_path,
                content_type=content_type,
                original_hash=original_hash,
                generated_content=generated_content,
                language=language,
                created_at=now,
                expires_at=expires_at
            )
            self.db.add(new_entry)
            self.db.commit()
            self.db.refresh(new_entry)
            return new_entry

    def get_user_cached_chapters(
        self,
        user_id: str
    ) -> List[PersonalizedContent]:
        """
        Get all cached content entries for a user.

        Args:
            user_id: The user's ID

        Returns:
            List of PersonalizedContent entries
        """
        return self.db.query(PersonalizedContent).filter(
            and_(
                PersonalizedContent.user_id == user_id,
                PersonalizedContent.expires_at > datetime.utcnow()
            )
        ).order_by(PersonalizedContent.created_at.desc()).all()

    def invalidate_cache(
        self,
        user_id: str,
        chapter_path: Optional[str] = None,
        content_type: Optional[str] = None
    ) -> int:
        """
        Invalidate (delete) cached content for a user.

        Args:
            user_id: The user's ID
            chapter_path: Optional specific chapter to invalidate
            content_type: Optional specific content type to invalidate

        Returns:
            Number of entries deleted
        """
        query = self.db.query(PersonalizedContent).filter(
            PersonalizedContent.user_id == user_id
        )

        if chapter_path:
            query = query.filter(PersonalizedContent.chapter_path == chapter_path)

        if content_type and content_type != "all":
            query = query.filter(PersonalizedContent.content_type == content_type)

        count = query.delete(synchronize_session=False)
        self.db.commit()
        return count

    def cleanup_expired(self) -> int:
        """
        Delete all expired cache entries.

        Returns:
            Number of entries deleted
        """
        count = self.db.query(PersonalizedContent).filter(
            PersonalizedContent.expires_at < datetime.utcnow()
        ).delete(synchronize_session=False)
        self.db.commit()
        return count

    def is_cache_valid(
        self,
        user_id: str,
        chapter_path: str,
        content_type: str,
        language: str,
        current_hash: str
    ) -> bool:
        """
        Check if cached content exists and is valid (not expired, hash matches).

        Args:
            user_id: The user's ID
            chapter_path: The chapter path
            content_type: Type of content
            language: Target language
            current_hash: Current MD5 hash of original content

        Returns:
            True if cache is valid, False otherwise
        """
        cached = self.get_cached_content(
            user_id, chapter_path, content_type, language
        )

        if not cached:
            return False

        return cached.matches_hash(current_hash)
