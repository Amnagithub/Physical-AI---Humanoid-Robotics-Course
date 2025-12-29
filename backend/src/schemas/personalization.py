"""
Pydantic schemas for personalization and translation API
Feature: 002-user-auth-personalization
"""

from datetime import datetime
from enum import Enum
from typing import Optional, List
from pydantic import BaseModel, Field


class BackgroundLevel(str, Enum):
    """User's experience level for software or hardware"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class ContentType(str, Enum):
    """Type of content transformation"""
    PERSONALIZED = "personalized"
    TRANSLATED = "translated"
    BOTH = "both"


class UserProfile(BaseModel):
    """User profile for personalization context"""
    software_background: BackgroundLevel = BackgroundLevel.BEGINNER
    hardware_background: BackgroundLevel = BackgroundLevel.BEGINNER


class PersonalizationRequest(BaseModel):
    """Request to personalize chapter content"""
    chapter_path: str = Field(..., pattern=r"^/docs/.*", description="Docusaurus document path")
    original_content: str = Field(..., min_length=1, description="Original markdown content")
    user_profile: UserProfile


class TranslationRequest(BaseModel):
    """Request to translate chapter content"""
    chapter_path: str = Field(..., pattern=r"^/docs/.*", description="Docusaurus document path")
    content: str = Field(..., min_length=1, description="Content to translate")
    target_language: str = Field(default="ur", pattern=r"^[a-z]{2}$", description="ISO 639-1 language code")


class ContentResponse(BaseModel):
    """Response containing generated content"""
    content: str = Field(..., description="Generated personalized or translated content")
    cached: bool = Field(..., description="Whether response was served from cache")
    generated_at: datetime = Field(..., description="When content was generated")
    expires_at: datetime = Field(..., description="When cached content expires")


class CachedContentResponse(BaseModel):
    """Response for cached content lookup"""
    cached: bool
    content_type: Optional[ContentType] = None
    language: Optional[str] = None
    content: Optional[str] = None
    generated_at: Optional[datetime] = None
    expires_at: Optional[datetime] = None
    original_hash: Optional[str] = None


class CachedChapter(BaseModel):
    """Summary of a cached chapter for user preferences"""
    chapter_path: str
    content_type: str
    language: str
    expires_at: datetime


class UserPreferences(BaseModel):
    """User's personalization preferences"""
    user_id: str
    software_background: BackgroundLevel
    hardware_background: BackgroundLevel
    preferred_language: str = "en"
    cached_chapters: List[CachedChapter] = []


class CacheInvalidationResponse(BaseModel):
    """Response for cache invalidation request"""
    invalidated: bool
    count: int = Field(..., description="Number of cache entries deleted")


class ErrorResponse(BaseModel):
    """Standard error response"""
    detail: str = Field(..., description="Human-readable error message")
    error_code: str = Field(..., description="Machine-readable error code")


class PersonalizedContentDB(BaseModel):
    """Database model representation"""
    id: str
    user_id: str
    chapter_path: str
    content_type: ContentType
    original_hash: str
    generated_content: str
    language: str
    created_at: datetime
    expires_at: datetime

    class Config:
        from_attributes = True
