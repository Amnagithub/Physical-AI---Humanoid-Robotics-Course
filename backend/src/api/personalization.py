"""
API endpoints for content personalization and translation
Feature: 002-user-auth-personalization
"""

import hashlib
import time
from collections import defaultdict
from fastapi import APIRouter, Depends, HTTPException, Header, Request
from sqlalchemy.orm import Session
from typing import Optional

from ..database import get_db
from ..repositories.personalization_repository import PersonalizationRepository
from ..services.personalization_service import personalization_service
from ..schemas.personalization import (
    PersonalizationRequest,
    TranslationRequest,
    ContentResponse,
    UserPreferences,
    CacheInvalidationResponse,
    CachedChapter,
    BackgroundLevel,
    UserProfile,
)


router = APIRouter()

# Simple in-memory rate limiter for personalization endpoints
# In production, use Redis or similar for distributed rate limiting
rate_limit_store: dict = defaultdict(lambda: {"count": 0, "reset_time": 0})
RATE_LIMIT_WINDOW = 60  # 1 minute
RATE_LIMIT_MAX_REQUESTS = 10  # 10 requests per minute per user


def check_rate_limit(user_id: str):
    """Check and update rate limit for a user"""
    now = time.time()
    record = rate_limit_store[user_id]

    if now > record["reset_time"]:
        record["count"] = 1
        record["reset_time"] = now + RATE_LIMIT_WINDOW
        return

    if record["count"] >= RATE_LIMIT_MAX_REQUESTS:
        raise HTTPException(
            status_code=429,
            detail="Rate limit exceeded. Please wait before making more requests."
        )

    record["count"] += 1


def get_user_id_from_header(x_user_id: Optional[str] = Header(None)) -> str:
    """
    Extract and validate user ID from request header.
    In production, this should validate against the auth server session.
    """
    if not x_user_id:
        raise HTTPException(
            status_code=401,
            detail="Authentication required. Please sign in to access personalization features."
        )
    return x_user_id


def compute_content_hash(content: str) -> str:
    """Compute MD5 hash of content for cache key"""
    return hashlib.md5(content.encode()).hexdigest()


@router.post("/content/personalize", response_model=ContentResponse)
def personalize_content(
    request: PersonalizationRequest,
    user_id: str = Depends(get_user_id_from_header),
    db: Session = Depends(get_db)
):
    """
    Personalize chapter content based on user's background levels.

    Checks cache first, generates new content if not cached or expired.
    """
    # Rate limiting
    check_rate_limit(user_id)

    repo = PersonalizationRepository(db)
    content_hash = compute_content_hash(request.original_content)

    # Check cache first
    cached = repo.get_cached_content(
        user_id=user_id,
        chapter_path=request.chapter_path,
        content_type="personalized",
        language="en"
    )

    if cached and cached.matches_hash(content_hash):
        return ContentResponse(
            content=cached.generated_content,
            cached=True,
            generated_at=cached.created_at,
            expires_at=cached.expires_at
        )

    # Generate new personalized content
    try:
        result = personalization_service.personalize_content(
            content=request.original_content,
            user_profile=request.user_profile
        )
    except Exception as e:
        raise HTTPException(
            status_code=503,
            detail=f"Personalization service unavailable: {str(e)}"
        )

    # Store in cache
    cached_entry = repo.store_cached_content(
        user_id=user_id,
        chapter_path=request.chapter_path,
        content_type="personalized",
        original_hash=content_hash,
        generated_content=result["content"],
        language="en"
    )

    return ContentResponse(
        content=result["content"],
        cached=False,
        generated_at=cached_entry.created_at,
        expires_at=cached_entry.expires_at
    )


@router.post("/content/translate", response_model=ContentResponse)
def translate_content(
    request: TranslationRequest,
    user_id: str = Depends(get_user_id_from_header),
    db: Session = Depends(get_db)
):
    """
    Translate chapter content to Urdu (or other supported language).

    Checks cache first, generates new translation if not cached or expired.
    """
    # Rate limiting
    check_rate_limit(user_id)

    repo = PersonalizationRepository(db)
    content_hash = compute_content_hash(request.content)

    # Check cache first
    cached = repo.get_cached_content(
        user_id=user_id,
        chapter_path=request.chapter_path,
        content_type="translated",
        language=request.target_language
    )

    if cached and cached.matches_hash(content_hash):
        return ContentResponse(
            content=cached.generated_content,
            cached=True,
            generated_at=cached.created_at,
            expires_at=cached.expires_at
        )

    # Generate new translation
    try:
        result = personalization_service.translate_content(
            content=request.content,
            target_language=request.target_language
        )
    except Exception as e:
        raise HTTPException(
            status_code=503,
            detail=f"Translation service unavailable: {str(e)}"
        )

    # Store in cache
    cached_entry = repo.store_cached_content(
        user_id=user_id,
        chapter_path=request.chapter_path,
        content_type="translated",
        original_hash=content_hash,
        generated_content=result["content"],
        language=request.target_language
    )

    return ContentResponse(
        content=result["content"],
        cached=False,
        generated_at=cached_entry.created_at,
        expires_at=cached_entry.expires_at
    )


@router.get("/content/user-preferences", response_model=UserPreferences)
def get_user_preferences(
    user_id: str = Depends(get_user_id_from_header),
    db: Session = Depends(get_db),
    x_software_background: Optional[str] = Header(None),
    x_hardware_background: Optional[str] = Header(None)
):
    """
    Get user's personalization preferences and cached chapters.

    Background levels are passed from the auth server via headers.
    """
    repo = PersonalizationRepository(db)

    # Get cached chapters for this user
    cached_entries = repo.get_user_cached_chapters(user_id)

    cached_chapters = [
        CachedChapter(
            chapter_path=entry.chapter_path,
            content_type=entry.content_type,
            language=entry.language,
            expires_at=entry.expires_at
        )
        for entry in cached_entries
    ]

    # Parse background levels from headers (with defaults)
    sw_bg = BackgroundLevel.BEGINNER
    hw_bg = BackgroundLevel.BEGINNER

    if x_software_background:
        try:
            sw_bg = BackgroundLevel(x_software_background.lower())
        except ValueError:
            pass

    if x_hardware_background:
        try:
            hw_bg = BackgroundLevel(x_hardware_background.lower())
        except ValueError:
            pass

    return UserPreferences(
        user_id=user_id,
        software_background=sw_bg,
        hardware_background=hw_bg,
        preferred_language="en",
        cached_chapters=cached_chapters
    )


@router.delete("/content/cache/{chapter_path:path}", response_model=CacheInvalidationResponse)
def invalidate_chapter_cache(
    chapter_path: str,
    user_id: str = Depends(get_user_id_from_header),
    db: Session = Depends(get_db),
    content_type: Optional[str] = None
):
    """
    Invalidate cached content for a specific chapter.

    Used when user updates their profile to get fresh personalization.
    """
    repo = PersonalizationRepository(db)

    # Ensure chapter_path starts with /docs/
    if not chapter_path.startswith("/docs/"):
        chapter_path = f"/docs/{chapter_path}"

    count = repo.invalidate_cache(
        user_id=user_id,
        chapter_path=chapter_path,
        content_type=content_type
    )

    return CacheInvalidationResponse(
        invalidated=count > 0,
        count=count
    )


@router.delete("/content/cache", response_model=CacheInvalidationResponse)
def invalidate_all_cache(
    user_id: str = Depends(get_user_id_from_header),
    db: Session = Depends(get_db)
):
    """
    Invalidate all cached content for the current user.

    Used when user significantly updates their profile.
    """
    repo = PersonalizationRepository(db)

    count = repo.invalidate_cache(user_id=user_id)

    return CacheInvalidationResponse(
        invalidated=count > 0,
        count=count
    )
