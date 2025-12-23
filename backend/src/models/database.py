from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, Float, ForeignKey, JSON, Boolean
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from datetime import datetime
import uuid
from typing import Optional

Base = declarative_base()

def generate_uuid():
    return str(uuid.uuid4())

class StudentSession(Base):
    __tablename__ = "student_sessions"

    id = Column(String, primary_key=True, default=generate_uuid)
    created_at = Column(DateTime, default=datetime.utcnow)
    last_activity = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    mode = Column(String, default='full_book')  # 'full_book' or 'selected_text'
    metadata_ = Column("metadata", JSON, default={})

    # Relationship
    questions = relationship("Question", back_populates="session", cascade="all, delete-orphan")

class Question(Base):
    __tablename__ = "questions"

    id = Column(String, primary_key=True, default=generate_uuid)
    session_id = Column(String, ForeignKey("student_sessions.id"), nullable=False)
    content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    selected_text = Column(Text, nullable=True)  # For selected text mode
    context_mode = Column(String, default='full_book')  # 'full_book' or 'selected_text'

    # Relationship
    session = relationship("StudentSession", back_populates="questions")
    answer = relationship("Answer", back_populates="question", uselist=False, cascade="all, delete-orphan")

class Answer(Base):
    __tablename__ = "answers"

    id = Column(String, primary_key=True, default=generate_uuid)
    question_id = Column(String, ForeignKey("questions.id"), nullable=False, unique=True)
    content = Column(Text, nullable=False)
    sources = Column(JSON, default=[])  # Array of source references
    created_at = Column(DateTime, default=datetime.utcnow)
    confidence_score = Column(Float, default=0.0)  # 0.0 to 1.0

    # Relationship
    question = relationship("Question", back_populates="answer")

class TextbookContent(Base):
    __tablename__ = "textbook_content"

    id = Column(String, primary_key=True, default=generate_uuid)
    section = Column(String, nullable=False)  # e.g., "Chapter 3.2"
    content = Column(Text, nullable=False)
    # Note: embedding would be stored separately in Qdrant, not in SQL
    metadata_ = Column("metadata", JSON, default={})
    created_at = Column(DateTime, default=datetime.utcnow)