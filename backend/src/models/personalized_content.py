"""
SQLAlchemy model for personalized content cache
Feature: 002-user-auth-personalization
"""

from sqlalchemy import Column, String, DateTime, Text, Index, CheckConstraint
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from datetime import datetime, timedelta
import uuid
from ..database import Base


class PersonalizedContent(Base):
    """
    Cache table for personalized and translated chapter content.

    Each entry stores generated content for a specific user, chapter, content type,
    and language combination. Entries expire after 7 days by default.
    """
    __tablename__ = "personalized_content"

    # Primary key
    id = Column(
        PostgresUUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4
    )

    # User who owns this cached content
    user_id = Column(String, nullable=False, index=True)

    # Chapter path (e.g., /docs/modules/ros2/chapter1)
    chapter_path = Column(String, nullable=False, index=True)

    # Type of content transformation
    content_type = Column(
        String,
        nullable=False,
        # Constraint is defined at table level below
    )

    # MD5 hash of original content for cache invalidation
    original_hash = Column(String(32), nullable=False)

    # The generated personalized/translated content
    generated_content = Column(Text, nullable=False)

    # Target language (ISO 639-1 code)
    language = Column(String(5), nullable=False, default="en")

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow)
    expires_at = Column(
        DateTime,
        default=lambda: datetime.utcnow() + timedelta(days=7)
    )

    # Table-level constraints
    __table_args__ = (
        # Ensure content_type is valid
        CheckConstraint(
            "content_type IN ('personalized', 'translated', 'both')",
            name="check_content_type"
        ),
        # Unique constraint for cache key
        Index(
            "unique_user_chapter_type_lang",
            "user_id", "chapter_path", "content_type", "language",
            unique=True
        ),
        # Index for expiration cleanup
        Index("idx_personalized_content_expires", "expires_at"),
    )

    def __repr__(self):
        return (
            f"<PersonalizedContent("
            f"id={self.id}, "
            f"user_id={self.user_id}, "
            f"chapter_path={self.chapter_path}, "
            f"content_type={self.content_type}, "
            f"language={self.language})>"
        )

    def is_expired(self) -> bool:
        """Check if this cache entry has expired"""
        return datetime.utcnow() > self.expires_at

    def matches_hash(self, content_hash: str) -> bool:
        """Check if the original content hash matches (cache still valid)"""
        return self.original_hash == content_hash

    @classmethod
    def cache_key(cls, user_id: str, chapter_path: str, content_type: str, language: str) -> tuple:
        """Generate cache key tuple for lookups"""
        return (user_id, chapter_path, content_type, language)
