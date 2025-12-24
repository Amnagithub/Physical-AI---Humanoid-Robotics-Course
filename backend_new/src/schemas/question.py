from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID


# Session schemas
class SessionCreateRequest(BaseModel):
    mode: str = "full_book"  # 'full_book' or 'selected_text'


class SessionResponse(BaseModel):
    id: UUID
    created_at: datetime
    mode: str

    class Config:
        from_attributes = True


# Question schemas
class QuestionCreateRequest(BaseModel):
    content: str
    selected_text: Optional[str] = None


class QuestionResponse(BaseModel):
    id: UUID
    content: str
    selected_text: Optional[str] = None

    class Config:
        from_attributes = True


# Answer schemas
class AnswerResponse(BaseModel):
    id: UUID
    content: str
    sources: List[str]
    confidence_score: Optional[float]

    class Config:
        from_attributes = True


# Combined response for question/answer
class QuestionAnswerResponse(BaseModel):
    question: QuestionResponse
    answer: AnswerResponse