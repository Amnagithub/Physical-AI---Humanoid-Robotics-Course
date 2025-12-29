# Research: User Authentication & Personalized Learning Layer

**Feature**: `002-user-auth-personalization`
**Date**: 2025-12-26
**Status**: Complete

## Research Summary

This document consolidates research findings for implementing Better Auth-based authentication with user profile extensions, content personalization, and Urdu translation in a Docusaurus-based course platform.

---

## 1. Architecture Decision: Better Auth Integration Pattern

### Decision
Use **Better Auth** as a standalone TypeScript authentication server, integrated via API routes with the existing Docusaurus frontend and a new dedicated auth backend service.

### Rationale
- Better Auth is framework-agnostic and provides native TypeScript support
- Supports email/password authentication out-of-the-box (FR-001, FR-002, FR-006)
- Has built-in session management with configurable expiry (FR-007)
- Supports custom `additionalFields` on user schema for profile data (FR-003, FR-004, FR-005)
- Can use PostgreSQL (Neon) as the database backend
- Provides React client hooks for seamless frontend integration

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| NextAuth/Auth.js | Tightly coupled to Next.js, Docusaurus uses React directly |
| Custom JWT implementation | More development effort, reinventing security-critical code |
| Firebase Auth | External dependency, less control over user schema |
| Passport.js | Requires more boilerplate, no built-in session management UI |

### Integration Pattern
```
┌─────────────────────────────────────────────────────────────────┐
│                      ARCHITECTURE                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐     ┌──────────────────────────────────┐  │
│  │  Docusaurus      │     │  Better Auth Server (Node.js)   │  │
│  │  Frontend        │────▶│  /api/auth/*                     │  │
│  │  (React)         │     │  - PostgreSQL (Neon)             │  │
│  └──────────────────┘     │  - User + Session tables         │  │
│           │               └──────────────────────────────────┘  │
│           │                           │                          │
│           ▼                           ▼                          │
│  ┌──────────────────┐     ┌──────────────────────────────────┐  │
│  │  RAG/Personalize │     │  FastAPI Backend                 │  │
│  │  Service         │◀───▶│  (Existing)                      │  │
│  │  (LLM Calls)     │     │  - Cohere Integration            │  │
│  └──────────────────┘     │  - Qdrant Vector DB              │  │
│                           └──────────────────────────────────┘  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Better Auth Configuration

### Decision
Configure Better Auth with email/password authentication and custom user fields for software/hardware background levels.

### Implementation Pattern

```typescript
// auth-server/src/auth.ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL, // Neon PostgreSQL
  }),

  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true, // Auto sign-in after signup
  },

  // Custom user fields for personalization (FR-003, FR-004, FR-005)
  user: {
    additionalFields: {
      softwareBackground: {
        type: ["beginner", "intermediate", "advanced"],
        required: false,
        defaultValue: "beginner",
      },
      hardwareBackground: {
        type: ["beginner", "intermediate", "advanced"],
        required: false,
        defaultValue: "beginner",
      },
      preferredLanguage: {
        type: "string",
        required: false,
        defaultValue: "en",
      },
    },
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update session every 24 hours
  },

  trustedOrigins: [
    "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app",
    "http://localhost:3000", // Development
  ],

  secret: process.env.BETTER_AUTH_SECRET,
  baseURL: process.env.BETTER_AUTH_URL,
});
```

### Client Setup (React/Docusaurus)

```typescript
// frontend/src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";
import { inferAdditionalFields } from "better-auth/client/plugins";

export const authClient = createAuthClient({
  baseURL: "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app",
  plugins: [
    inferAdditionalFields({
      user: {
        softwareBackground: { type: "string", required: false },
        hardwareBackground: { type: "string", required: false },
        preferredLanguage: { type: "string", required: false },
      },
    }),
  ],
});

// Export hooks
export const { useSession, signIn, signUp, signOut } = authClient;
```

---

## 3. Docusaurus Integration Strategy

### Decision
Use Docusaurus's theme swizzling to inject authentication components and chapter personalization controls.

### Rationale
- Docusaurus supports React component swizzling via `@theme` aliases
- The `Root.js` component wraps the entire site (already used for FloatingChatButton)
- Custom components can be added to `src/components/` and imported into swizzled theme components

### Implementation Approach

1. **Root.js Enhancement** - Add AuthProvider context
```typescript
// frontend/src/theme/Root.js
import React from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';
import FloatingChatButton from '@site/src/components/FloatingChatButton';

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <FloatingChatButton apiBaseUrl={API_BASE_URL} />
    </AuthProvider>
  );
}
```

2. **Swizzle DocItem** - Add personalization controls to chapter pages
```bash
npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap
```

3. **Custom Components**
- `AuthModal.jsx` - Login/Signup modal
- `UserProfileButton.jsx` - Navbar user button
- `ChapterPersonalization.jsx` - Per-chapter personalize/translate controls

---

## 4. Database Schema Design

### Decision
Extend Better Auth's default tables with custom user fields; add a separate table for caching personalized content.

### Schema

```sql
-- Better Auth manages these tables automatically
-- We configure additionalFields which adds columns to the user table

-- User table (managed by Better Auth + additionalFields)
-- id, name, email, emailVerified, image, createdAt, updatedAt
-- + softwareBackground (enum: beginner/intermediate/advanced)
-- + hardwareBackground (enum: beginner/intermediate/advanced)
-- + preferredLanguage (string, default: 'en')

-- Session table (managed by Better Auth)
-- id, userId, token, expiresAt, createdAt, ipAddress, userAgent

-- Custom table for caching personalized/translated content
CREATE TABLE personalized_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  chapter_path TEXT NOT NULL,
  content_type TEXT NOT NULL CHECK (content_type IN ('personalized', 'translated', 'both')),
  original_hash TEXT NOT NULL, -- MD5 of original content for cache invalidation
  generated_content TEXT NOT NULL,
  language TEXT DEFAULT 'en',
  created_at TIMESTAMP DEFAULT NOW(),
  expires_at TIMESTAMP DEFAULT NOW() + INTERVAL '7 days',
  UNIQUE(user_id, chapter_path, content_type, language)
);

CREATE INDEX idx_personalized_content_user ON personalized_content(user_id);
CREATE INDEX idx_personalized_content_chapter ON personalized_content(chapter_path);
```

---

## 5. Personalization & Translation Service

### Decision
Extend the existing FastAPI backend with new endpoints for content personalization and translation, leveraging the existing Cohere integration.

### Rationale
- Reuses existing LLM infrastructure (Cohere)
- Keeps personalization logic server-side for security
- Can implement caching to reduce LLM costs

### API Design

```python
# POST /api/v1/content/personalize
{
  "chapter_path": "/docs/modules/ros2-nervous-system/chapter1-intro",
  "original_content": "...",  # Markdown content
  "user_profile": {
    "software_background": "advanced",
    "hardware_background": "beginner"
  }
}
# Response: { "personalized_content": "..." }

# POST /api/v1/content/translate
{
  "chapter_path": "/docs/modules/ros2-nervous-system/chapter1-intro",
  "content": "...",  # Content to translate (can be original or personalized)
  "target_language": "ur"  # Urdu
}
# Response: { "translated_content": "..." }
```

### LLM Prompt Strategy

**Personalization Prompt:**
```
You are an expert technical writer adapting course content for a learner.

Learner Profile:
- Software/Programming Level: {software_background}
- Hardware/Electronics Level: {hardware_background}

Task: Adapt the following course content to match the learner's background.
- For beginners: Add more explanations, analogies, and step-by-step breakdowns
- For intermediate: Balance theory with practical examples
- For advanced: Focus on technical depth, edge cases, and advanced patterns

Preserve all code blocks, images, and structural elements.
Do not add or remove topics - only adjust explanation depth and examples.

Original Content:
{content}
```

**Translation Prompt:**
```
You are a professional technical translator specializing in Urdu.

Task: Translate the following technical course content to Urdu.
- Preserve all code blocks exactly as-is (do not translate code)
- Preserve all markdown formatting
- Translate technical terms accurately with original English term in parentheses
- Maintain the same structure and headings

Content:
{content}
```

---

## 6. Frontend State Management

### Decision
Use React Context for auth state with Better Auth's `useSession` hook as the source of truth.

### Implementation

```typescript
// frontend/src/context/AuthContext.tsx
import React, { createContext, useContext, ReactNode } from 'react';
import { authClient } from '@site/src/lib/auth-client';

interface AuthContextType {
  session: any;
  user: any;
  isLoading: boolean;
  signIn: typeof authClient.signIn;
  signUp: typeof authClient.signUp;
  signOut: typeof authClient.signOut;
}

const AuthContext = createContext<AuthContextType | null>(null);

export function AuthProvider({ children }: { children: ReactNode }) {
  const { data: session, isPending: isLoading } = authClient.useSession();

  return (
    <AuthContext.Provider value={{
      session: session?.session,
      user: session?.user,
      isLoading,
      signIn: authClient.signIn,
      signUp: authClient.signUp,
      signOut: authClient.signOut,
    }}>
      {children}
    </AuthContext.Provider>
  );
}

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
};
```

---

## 7. Security Considerations

### Decisions

1. **Credentials Storage**: Better Auth handles password hashing (bcrypt by default)
2. **Session Security**: HTTP-only cookies with secure flag in production
3. **CORS**: Restrict to known origins only
4. **Rate Limiting**: Implement on auth endpoints (signup: 5/min, signin: 10/min)
5. **Input Validation**: Zod schemas on all API endpoints
6. **Content Integrity**: Original markdown files are NEVER modified (FR-015)

### Environment Variables
```env
# Already provided
BETTER_AUTH_SECRET=e8o8oJh/rFV7IafAGRKzzlxELIOHzGf27U3H4FJNe5E=
BETTER_AUTH_URL=https://physical-ai-humanoid-robotics-cours-ashen.vercel.app

# Additional required
DATABASE_URL=postgresql://... (existing Neon connection)
COHERE_API_KEY=... (existing)
```

---

## 8. Performance Considerations

### Decisions

1. **Content Caching**: Cache personalized/translated content for 7 days per user/chapter
2. **Cache Invalidation**: Hash original content; regenerate if hash changes
3. **Lazy Loading**: Auth components loaded only when needed
4. **Streaming**: Consider streaming LLM responses for faster perceived performance

### Success Metrics Alignment
- SC-001: Signup < 2 min ✓ (simple form)
- SC-003/SC-004: Personalization/Translation < 10s ✓ (with caching, first-time may be 5-8s)
- SC-005: 100% reversible ✓ (original content preserved)
- SC-007: Zero overwrites ✓ (generated content stored separately)

---

## 9. Open Questions Resolved

| Question | Resolution |
|----------|------------|
| How to run Better Auth with Docusaurus? | Separate Node.js server for auth API, Docusaurus calls it via HTTP |
| Where to host auth server? | Same Vercel deployment, serverless functions or separate service |
| How to share database with FastAPI backend? | Same Neon PostgreSQL, different tables |
| How to inject auth into Docusaurus pages? | Theme swizzling (Root.js, DocItem) |
| How to protect personalization features? | Check session on frontend; validate user on API calls |

---

## 10. Technology Stack Summary

| Component | Technology | Notes |
|-----------|------------|-------|
| Auth Server | Better Auth + Node.js | TypeScript, Hono or Express adapter |
| Auth Client | better-auth/react | React hooks for Docusaurus |
| Database | Neon PostgreSQL | Shared with FastAPI backend |
| Frontend | Docusaurus 3.9.2 | React 18, theme swizzling |
| LLM | Cohere (existing) | command-r-plus for personalization |
| Deployment | Vercel | Frontend + serverless auth API |

---

## References

- Better Auth Documentation: https://better-auth.com
- Better Auth GitHub: https://github.com/better-auth/better-auth
- Docusaurus Swizzling: https://docusaurus.io/docs/swizzling
- Feature Spec: `specs/002-user-auth-personalization/spec.md`
