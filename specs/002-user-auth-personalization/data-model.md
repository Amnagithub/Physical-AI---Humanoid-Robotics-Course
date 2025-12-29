# Data Model: User Authentication & Personalized Learning Layer

**Feature**: `002-user-auth-personalization`
**Date**: 2025-12-26

---

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          DATA MODEL                                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────┐         ┌──────────────────────────┐          │
│  │        user          │         │        session           │          │
│  ├──────────────────────┤         ├──────────────────────────┤          │
│  │ id (PK)              │────────▶│ id (PK)                  │          │
│  │ email (UNIQUE)       │         │ userId (FK → user)       │          │
│  │ emailVerified        │         │ token                    │          │
│  │ name                 │         │ expiresAt                │          │
│  │ image                │         │ ipAddress                │          │
│  │ createdAt            │         │ userAgent                │          │
│  │ updatedAt            │         │ createdAt                │          │
│  │ ─────────────────────│         │ updatedAt                │          │
│  │ softwareBackground   │         └──────────────────────────┘          │
│  │ hardwareBackground   │                                               │
│  │ preferredLanguage    │                                               │
│  └──────────────────────┘                                               │
│            │                                                             │
│            │                                                             │
│            ▼                                                             │
│  ┌──────────────────────────────────────────────────────────┐           │
│  │              personalized_content                         │           │
│  ├──────────────────────────────────────────────────────────┤           │
│  │ id (PK)                                                   │           │
│  │ userId (FK → user)                                        │           │
│  │ chapterPath                                               │           │
│  │ contentType (personalized | translated | both)            │           │
│  │ originalHash                                              │           │
│  │ generatedContent                                          │           │
│  │ language                                                  │           │
│  │ createdAt                                                 │           │
│  │ expiresAt                                                 │           │
│  │ UNIQUE(userId, chapterPath, contentType, language)        │           │
│  └──────────────────────────────────────────────────────────┘           │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Entity Definitions

### 1. User (Managed by Better Auth + Custom Fields)

The `user` table is automatically created and managed by Better Auth. We extend it with `additionalFields` for personalization.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | TEXT | PRIMARY KEY | Unique identifier (UUID) |
| `email` | TEXT | NOT NULL, UNIQUE | User's email address |
| `emailVerified` | BOOLEAN | DEFAULT FALSE | Email verification status |
| `name` | TEXT | NOT NULL | User's display name |
| `image` | TEXT | NULLABLE | Profile image URL |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Account creation timestamp |
| `updatedAt` | TIMESTAMP | AUTO UPDATE | Last update timestamp |
| `softwareBackground` | TEXT | DEFAULT 'beginner' | Software/programming experience level |
| `hardwareBackground` | TEXT | DEFAULT 'beginner' | Hardware/electronics experience level |
| `preferredLanguage` | TEXT | DEFAULT 'en' | Preferred content language |

**Validation Rules:**
- `email`: Must be valid email format, case-insensitive uniqueness
- `softwareBackground`: Must be one of: `beginner`, `intermediate`, `advanced`
- `hardwareBackground`: Must be one of: `beginner`, `intermediate`, `advanced`
- `preferredLanguage`: ISO 639-1 language code (e.g., `en`, `ur`)

**State Transitions:**
- `emailVerified`: `false` → `true` (after email verification, if implemented in future)

---

### 2. Session (Managed by Better Auth)

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | TEXT | PRIMARY KEY | Session identifier |
| `userId` | TEXT | FK → user(id), NOT NULL | Associated user |
| `token` | TEXT | NOT NULL, UNIQUE | Session token |
| `expiresAt` | TIMESTAMP | NOT NULL | Session expiration time |
| `ipAddress` | TEXT | NULLABLE | Client IP address |
| `userAgent` | TEXT | NULLABLE | Client user agent |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Session creation time |
| `updatedAt` | TIMESTAMP | AUTO UPDATE | Last activity time |

**Validation Rules:**
- Session expires after 7 days of inactivity
- Token is refreshed every 24 hours of activity

---

### 3. Personalized Content (Custom Table)

Caches generated personalized and/or translated content per user per chapter.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY | Unique identifier |
| `userId` | TEXT | FK → user(id), NOT NULL | Content owner |
| `chapterPath` | TEXT | NOT NULL | Docusaurus doc path (e.g., `/docs/modules/ros2/chapter1`) |
| `contentType` | TEXT | NOT NULL | Type: `personalized`, `translated`, `both` |
| `originalHash` | TEXT | NOT NULL | MD5 hash of original content for cache invalidation |
| `generatedContent` | TEXT | NOT NULL | The personalized/translated markdown |
| `language` | TEXT | DEFAULT 'en' | Target language code |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Cache creation time |
| `expiresAt` | TIMESTAMP | DEFAULT NOW() + 7 days | Cache expiration |

**Constraints:**
- `UNIQUE(userId, chapterPath, contentType, language)` - One cache entry per user/chapter/type/language combination
- `CHECK (contentType IN ('personalized', 'translated', 'both'))`

**Indexes:**
- `idx_personalized_content_user` ON `userId`
- `idx_personalized_content_chapter` ON `chapterPath`
- `idx_personalized_content_expires` ON `expiresAt` (for cleanup jobs)

**Validation Rules:**
- `chapterPath`: Must match pattern `/docs/**/*`
- `originalHash`: 32-character MD5 hex string
- `language`: Valid ISO 639-1 code

**State Transitions:**
- Cache entry created on first personalization/translation request
- Cache invalidated when `originalHash` doesn't match current content
- Cache expires after 7 days (soft delete via `expiresAt`)

---

## SQL Schema

```sql
-- Better Auth tables are auto-created via CLI: npx better-auth generate

-- Custom table for content caching
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

  CONSTRAINT fk_user FOREIGN KEY (user_id) REFERENCES "user"(id) ON DELETE CASCADE,
  CONSTRAINT unique_user_chapter_type_lang UNIQUE (user_id, chapter_path, content_type, language)
);

-- Indexes for query performance
CREATE INDEX IF NOT EXISTS idx_personalized_content_user
  ON personalized_content(user_id);

CREATE INDEX IF NOT EXISTS idx_personalized_content_chapter
  ON personalized_content(chapter_path);

CREATE INDEX IF NOT EXISTS idx_personalized_content_expires
  ON personalized_content(expires_at);

-- Cleanup function for expired cache entries (run via cron)
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
```

---

## TypeScript Types

```typescript
// types/auth.ts

export type BackgroundLevel = 'beginner' | 'intermediate' | 'advanced';

export interface User {
  id: string;
  email: string;
  emailVerified: boolean;
  name: string;
  image: string | null;
  createdAt: Date;
  updatedAt: Date;
  // Custom fields
  softwareBackground: BackgroundLevel;
  hardwareBackground: BackgroundLevel;
  preferredLanguage: string;
}

export interface Session {
  id: string;
  userId: string;
  token: string;
  expiresAt: Date;
  ipAddress: string | null;
  userAgent: string | null;
  createdAt: Date;
  updatedAt: Date;
}

export interface UserSession {
  user: User;
  session: Session;
}

// types/content.ts

export type ContentType = 'personalized' | 'translated' | 'both';

export interface PersonalizedContent {
  id: string;
  userId: string;
  chapterPath: string;
  contentType: ContentType;
  originalHash: string;
  generatedContent: string;
  language: string;
  createdAt: Date;
  expiresAt: Date;
}

export interface PersonalizationRequest {
  chapterPath: string;
  originalContent: string;
  userProfile: {
    softwareBackground: BackgroundLevel;
    hardwareBackground: BackgroundLevel;
  };
}

export interface TranslationRequest {
  chapterPath: string;
  content: string;
  targetLanguage: string; // 'ur' for Urdu
}

export interface ContentResponse {
  content: string;
  cached: boolean;
  generatedAt: string;
  expiresAt: string;
}
```

---

## Pydantic Models (FastAPI)

```python
# backend/src/schemas/personalization.py

from datetime import datetime
from enum import Enum
from typing import Optional
from pydantic import BaseModel, Field

class BackgroundLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class ContentType(str, Enum):
    PERSONALIZED = "personalized"
    TRANSLATED = "translated"
    BOTH = "both"

class UserProfile(BaseModel):
    software_background: BackgroundLevel = BackgroundLevel.BEGINNER
    hardware_background: BackgroundLevel = BackgroundLevel.BEGINNER

class PersonalizationRequest(BaseModel):
    chapter_path: str = Field(..., pattern=r"^/docs/.*")
    original_content: str = Field(..., min_length=1)
    user_profile: UserProfile

class TranslationRequest(BaseModel):
    chapter_path: str = Field(..., pattern=r"^/docs/.*")
    content: str = Field(..., min_length=1)
    target_language: str = Field(default="ur", pattern=r"^[a-z]{2}$")

class ContentResponse(BaseModel):
    content: str
    cached: bool
    generated_at: datetime
    expires_at: datetime

class PersonalizedContentDB(BaseModel):
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
```

---

## Data Flow Diagrams

### Signup Flow
```
User → SignupForm → authClient.signUp.email({
  name, email, password,
  softwareBackground,  // Custom field
  hardwareBackground   // Custom field
}) → Better Auth Server → PostgreSQL (user table)
    ↓
Session created → Cookie set → Redirect to /docs
```

### Personalization Flow
```
User clicks "Personalize" on Chapter Page
    ↓
Frontend reads chapter markdown (from DOM or fetch)
    ↓
POST /api/v1/content/personalize {
  chapterPath,
  originalContent,
  userProfile: { softwareBackground, hardwareBackground }
}
    ↓
Backend checks cache (personalized_content table)
    ├── Cache hit & valid hash → Return cached content
    └── Cache miss or stale →
        ↓
        Generate via Cohere LLM
        ↓
        Store in personalized_content table
        ↓
        Return generated content
    ↓
Frontend displays personalized content (preserves original in state)
```

### Translation Flow
```
User clicks "Translate to Urdu" on Chapter Page
    ↓
Frontend gets current content (original or personalized)
    ↓
POST /api/v1/content/translate {
  chapterPath,
  content,
  targetLanguage: "ur"
}
    ↓
Backend checks cache
    ├── Cache hit → Return cached translation
    └── Cache miss →
        ↓
        Translate via Cohere LLM
        ↓
        Store in personalized_content table
        ↓
        Return translated content
    ↓
Frontend displays Urdu content
```

---

## Migration Strategy

### Phase 1: Better Auth Setup
1. Run `npx better-auth generate` to create auth tables
2. Tables created: `user`, `session`, `account`, `verification`

### Phase 2: Custom Fields Migration
Better Auth CLI automatically adds custom columns when `additionalFields` are configured.

### Phase 3: Personalized Content Table
Run migration script to create `personalized_content` table.

### Rollback Plan
- Better Auth tables: `DROP TABLE session, account, verification, "user" CASCADE;`
- Custom table: `DROP TABLE personalized_content;`
