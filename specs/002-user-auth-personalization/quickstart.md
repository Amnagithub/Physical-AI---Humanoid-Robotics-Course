# Quickstart: User Authentication & Personalized Learning

**Feature**: `002-user-auth-personalization`
**Estimated Setup Time**: 30-45 minutes

---

## Prerequisites

- Node.js 18+
- Python 3.11+
- PostgreSQL (Neon account already configured)
- Existing project cloned and dependencies installed

---

## Quick Setup Steps

### 1. Install Better Auth Dependencies

```bash
# In the auth-server directory (to be created)
cd auth-server
npm init -y
npm install better-auth pg hono @hono/node-server dotenv
npm install -D typescript @types/node tsx
```

### 2. Create Auth Server Configuration

```typescript
// auth-server/src/auth.ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
  },
  user: {
    additionalFields: {
      softwareBackground: {
        type: ["beginner", "intermediate", "advanced"],
        defaultValue: "beginner",
      },
      hardwareBackground: {
        type: ["beginner", "intermediate", "advanced"],
        defaultValue: "beginner",
      },
      preferredLanguage: {
        type: "string",
        defaultValue: "en",
      },
    },
  },
  trustedOrigins: [
    process.env.BETTER_AUTH_URL!,
    "http://localhost:3000",
  ],
  secret: process.env.BETTER_AUTH_SECRET,
});
```

### 3. Create Auth API Server

```typescript
// auth-server/src/index.ts
import { Hono } from "hono";
import { cors } from "hono/cors";
import { serve } from "@hono/node-server";
import { auth } from "./auth";

const app = new Hono();

app.use("/*", cors({
  origin: [process.env.BETTER_AUTH_URL!, "http://localhost:3000"],
  credentials: true,
}));

app.on(["POST", "GET"], "/api/auth/*", (c) => auth.handler(c.req.raw));

serve({ fetch: app.fetch, port: 3001 }, () => {
  console.log("Auth server running on http://localhost:3001");
});
```

### 4. Generate Database Schema

```bash
npx better-auth generate
npx better-auth migrate
```

### 5. Install Frontend Auth Client

```bash
# In the frontend directory
cd frontend
npm install better-auth
```

### 6. Create Auth Client

```typescript
// frontend/src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
});

export const { useSession, signIn, signUp, signOut } = authClient;
```

### 7. Create Auth Context

```jsx
// frontend/src/context/AuthContext.jsx
import React, { createContext, useContext } from 'react';
import { authClient } from '../lib/auth-client';

const AuthContext = createContext(null);

export function AuthProvider({ children }) {
  const { data: sessionData, isPending } = authClient.useSession();

  return (
    <AuthContext.Provider value={{
      user: sessionData?.user,
      session: sessionData?.session,
      isLoading: isPending,
      isAuthenticated: !!sessionData?.user,
      signIn: authClient.signIn,
      signUp: authClient.signUp,
      signOut: authClient.signOut,
    }}>
      {children}
    </AuthContext.Provider>
  );
}

export const useAuth = () => useContext(AuthContext);
```

### 8. Update Root.js

```jsx
// frontend/src/theme/Root.js
import React from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';
import FloatingChatButton from '@site/src/components/FloatingChatButton';

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <FloatingChatButton apiBaseUrl="http://localhost:8000" />
    </AuthProvider>
  );
}
```

---

## Environment Variables

Create/update `.env` files:

```bash
# auth-server/.env
DATABASE_URL=postgresql://...  # Your Neon connection string
BETTER_AUTH_SECRET=e8o8oJh/rFV7IafAGRKzzlxELIOHzGf27U3H4FJNe5E=
BETTER_AUTH_URL=https://physical-ai-humanoid-robotics-cours-ashen.vercel.app

# frontend/.env
BETTER_AUTH_URL=https://physical-ai-humanoid-robotics-cours-ashen.vercel.app
```

---

## Verification Steps

### 1. Test Auth Server

```bash
# Start auth server
cd auth-server && npm run dev

# Test signup
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Test User",
    "email": "test@example.com",
    "password": "password123",
    "softwareBackground": "beginner",
    "hardwareBackground": "beginner"
  }'

# Expected: 200 OK with user and session data
```

### 2. Test Frontend Integration

```bash
# Start Docusaurus
cd frontend && npm start

# Visit http://localhost:3000
# Should load without errors
# Auth context should be available
```

### 3. Test Database Schema

```sql
-- Connect to your Neon database
-- Verify tables exist
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public'
AND table_name IN ('user', 'session', 'account', 'verification');

-- Verify custom fields
SELECT column_name FROM information_schema.columns
WHERE table_name = 'user'
AND column_name IN ('softwareBackground', 'hardwareBackground', 'preferredLanguage');
```

---

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| CORS errors | Ensure `trustedOrigins` includes your frontend URL |
| Session not persisting | Check cookie settings, ensure `credentials: 'include'` |
| Custom fields not appearing | Run `npx better-auth migrate` after config changes |
| Database connection fails | Verify `DATABASE_URL` format and Neon project status |

---

## Next Steps

After basic auth is working:

1. Implement signup/login UI components
2. Add chapter personalization controls
3. Implement FastAPI personalization endpoints
4. Add Urdu translation functionality
5. Deploy to Vercel

See `tasks.md` for detailed implementation tasks.
