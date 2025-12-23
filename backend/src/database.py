from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from contextlib import asynccontextmanager
from typing import AsyncGenerator

# Get database URL from environment
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql+asyncpg://username:password@localhost:5432/rag_chatbot")

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=5,
    max_overflow=10,
    pool_pre_ping=True,
    pool_recycle=300,
)

# Create async session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=async_sessionmaker,
    expire_on_commit=False
)

# For sync operations (migrations, etc.)
sync_engine = create_engine(
    DATABASE_URL.replace("+asyncpg", ""),
    poolclass=QueuePool,
    pool_size=5,
    max_overflow=10,
    pool_pre_ping=True,
    pool_recycle=300,
)

SessionLocal = sessionmaker(bind=sync_engine, expire_on_commit=False)

async def get_db() -> AsyncGenerator:
    """Dependency for getting database session"""
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()

# Function to initialize the database tables
async def init_db():
    """Initialize database tables based on models"""
    from .models.database import Base
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

# Function to get sync database session for migrations
def get_sync_db():
    """Get synchronous database session for migrations and sync operations"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()