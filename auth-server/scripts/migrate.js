import pg from 'pg';
import 'dotenv/config';

const { Pool } = pg;

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

const schema = `
-- Users table (with camelCase column names for Better Auth)
CREATE TABLE IF NOT EXISTS "user" (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    email TEXT NOT NULL UNIQUE,
    "emailVerified" BOOLEAN NOT NULL DEFAULT FALSE,
    image TEXT,
    "createdAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    "softwareBackground" TEXT DEFAULT 'beginner',
    "hardwareBackground" TEXT DEFAULT 'beginner',
    "preferredLanguage" TEXT DEFAULT 'en'
);

-- Accounts table
CREATE TABLE IF NOT EXISTS "account" (
    id TEXT PRIMARY KEY,
    "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    "accountId" TEXT NOT NULL,
    "providerId" TEXT NOT NULL,
    "accessToken" TEXT,
    "refreshToken" TEXT,
    "accessTokenExpiresAt" TIMESTAMP WITH TIME ZONE,
    "refreshTokenExpiresAt" TIMESTAMP WITH TIME ZONE,
    password TEXT,
    scope TEXT,
    "idToken" TEXT,
    "createdAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    UNIQUE("providerId", "accountId")
);

-- Sessions table
CREATE TABLE IF NOT EXISTS "session" (
    id TEXT PRIMARY KEY,
    "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    token TEXT NOT NULL UNIQUE,
    "expiresAt" TIMESTAMP WITH TIME ZONE NOT NULL,
    "ipAddress" TEXT,
    "userAgent" TEXT,
    "createdAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

-- Verification tokens table
CREATE TABLE IF NOT EXISTS "verification" (
    id TEXT PRIMARY KEY,
    identifier TEXT NOT NULL,
    token TEXT NOT NULL,
    "expiresAt" TIMESTAMP WITH TIME ZONE NOT NULL,
    "createdAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    UNIQUE(identifier, token)
);

-- Create indexes
CREATE INDEX IF NOT EXISTS idx_user_email ON "user"(email);
CREATE INDEX IF NOT EXISTS idx_session_user_id ON "session"("userId");
CREATE INDEX IF NOT EXISTS idx_session_token ON "session"(token);
CREATE INDEX IF NOT EXISTS idx_account_user_id ON "account"("userId");

-- Personalized content cache table for user personalization/translation
CREATE TABLE IF NOT EXISTS personalized_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id TEXT NOT NULL,
    chapter_path TEXT NOT NULL,
    content_type TEXT NOT NULL,
    original_hash TEXT NOT NULL,
    generated_content TEXT NOT NULL,
    language TEXT NOT NULL DEFAULT 'en',
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW() + INTERVAL '7 days',
    CONSTRAINT check_content_type CHECK (content_type IN ('personalized', 'translated', 'both'))
);

CREATE INDEX IF NOT EXISTS idx_personalized_content_user_id ON personalized_content(user_id);
CREATE INDEX IF NOT EXISTS idx_personalized_content_chapter_path ON personalized_content(chapter_path);
CREATE INDEX IF NOT EXISTS idx_personalized_content_expires ON personalized_content(expires_at);

-- Session table for chat sessions
CREATE TABLE IF NOT EXISTS chat_session (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id TEXT NOT NULL,
    mode TEXT NOT NULL DEFAULT 'full_book',
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_chat_session_user_id ON chat_session(user_id);

-- Question table
CREATE TABLE IF NOT EXISTS question (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_session(id) ON DELETE CASCADE,
    content TEXT NOT NULL,
    selected_text TEXT,
    context_mode TEXT NOT NULL DEFAULT 'full_book',
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_question_session_id ON question(session_id);

-- Answer table
CREATE TABLE IF NOT EXISTS answer (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    question_id UUID NOT NULL REFERENCES question(id) ON DELETE CASCADE,
    content TEXT NOT NULL,
    sources TEXT[] DEFAULT '{}',
    confidence_score REAL DEFAULT 0.0,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_answer_question_id ON answer(question_id);
`;

async function runMigration() {
  console.log('Connecting to database...');
  const client = await pool.connect();

  try {
    console.log('Dropping existing tables for clean migration...');
    await client.query('DROP TABLE IF EXISTS "verification" CASCADE');
    await client.query('DROP TABLE IF EXISTS "session" CASCADE');
    await client.query('DROP TABLE IF EXISTS "account" CASCADE');
    await client.query('DROP TABLE IF EXISTS "user" CASCADE');

    console.log('Running migration...');
    await client.query(schema);
    console.log('Migration completed successfully!');
  } catch (error) {
    console.error('Migration failed:', error.message);
    process.exit(1);
  } finally {
    client.release();
    await pool.end();
  }
}

runMigration();
