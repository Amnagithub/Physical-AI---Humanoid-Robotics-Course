-- Migration: Create personalized_content table
-- Feature: 002-user-auth-personalization
-- Date: 2025-12-26

-- Custom table for caching personalized and translated content
CREATE TABLE IF NOT EXISTS personalized_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id TEXT NOT NULL,
  chapter_path TEXT NOT NULL,
  content_type TEXT NOT NULL CHECK (content_type IN ('personalized', 'translated', 'both')),
  original_hash CHAR(32) NOT NULL,
  generated_content TEXT NOT NULL,
  language VARCHAR(5) NOT NULL DEFAULT 'en',
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  expires_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() + INTERVAL '7 days',

  -- Unique constraint: one cache entry per user/chapter/type/language
  CONSTRAINT unique_user_chapter_type_lang UNIQUE (user_id, chapter_path, content_type, language)
);

-- Index for fast user lookups
CREATE INDEX IF NOT EXISTS idx_personalized_content_user
  ON personalized_content(user_id);

-- Index for chapter path lookups
CREATE INDEX IF NOT EXISTS idx_personalized_content_chapter
  ON personalized_content(chapter_path);

-- Index for cache expiration cleanup
CREATE INDEX IF NOT EXISTS idx_personalized_content_expires
  ON personalized_content(expires_at);

-- Cleanup function for expired cache entries (can be run via cron job)
CREATE OR REPLACE FUNCTION cleanup_expired_personalized_content()
RETURNS INTEGER AS $$
DECLARE
  deleted_count INTEGER;
BEGIN
  DELETE FROM personalized_content WHERE expires_at < NOW();
  GET DIAGNOSTICS deleted_count = ROW_COUNT;
  RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- Add comment for documentation
COMMENT ON TABLE personalized_content IS 'Cache for personalized and translated chapter content per user';
COMMENT ON COLUMN personalized_content.content_type IS 'Type of transformation: personalized, translated, or both';
COMMENT ON COLUMN personalized_content.original_hash IS 'MD5 hash of original content for cache invalidation';
COMMENT ON COLUMN personalized_content.expires_at IS 'Cache entries expire after 7 days by default';
