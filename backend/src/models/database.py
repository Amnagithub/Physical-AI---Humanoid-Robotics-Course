from sqlalchemy import Column, String, DateTime, Text, Float, UUID, JSON, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.ext.mutable import MutableDict
from datetime import datetime
import uuid
from ..database import Base


class UserSession(Base):
    __tablename__ = "user_sessions"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at = Column(DateTime, default=datetime.utcnow)
    last_activity = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    mode = Column(String, default="full_book")  # 'full_book' or 'selected_text'
    metadata_ = Column("metadata", MutableDict.as_mutable(JSON), default={})


class Question(Base):
    __tablename__ = "questions"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PostgresUUID(as_uuid=True), ForeignKey("user_sessions.id"), nullable=False)
    content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    selected_text = Column(Text, nullable=True)
    context_mode = Column(String, default="full_book")  # 'full_book' or 'selected_text'


class Answer(Base):
    __tablename__ = "answers"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    question_id = Column(PostgresUUID(as_uuid=True), ForeignKey("questions.id"), nullable=False)
    content = Column(Text, nullable=False)
    sources = Column(JSON, default=list)  # Array of source references
    created_at = Column(DateTime, default=datetime.utcnow)
    confidence_score = Column(Float)


class BookContent(Base):
    __tablename__ = "book_content"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    section = Column(String, nullable=False)
    content = Column(Text, nullable=False)
    # Note: embedding would typically be stored in Qdrant, not Postgres
    metadata_ = Column("metadata", MutableDict.as_mutable(JSON), default={})  # book_title, chapter, section, page_range
    created_at = Column(DateTime, default=datetime.utcnow)