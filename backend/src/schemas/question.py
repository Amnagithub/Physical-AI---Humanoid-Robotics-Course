from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class QuestionCreate(BaseModel):
    content: str
    selected_text: Optional[str] = None
    mode: Optional[str] = None  # 'full_book' or 'selected_text'


class QuestionResponse(BaseModel):
    id: str
    session_id: str
    content: str
    created_at: str
    selected_text: Optional[str] = None
    context_mode: str

    class Config:
        from_attributes = True


class AnswerResponse(BaseModel):
    id: str
    question_id: str
    content: str
    sources: List[str]
    created_at: str
    confidence_score: float

    class Config:
        from_attributes = True