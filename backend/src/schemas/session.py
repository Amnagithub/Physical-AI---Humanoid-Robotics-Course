from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class SessionCreateRequest(BaseModel):
    mode: str = "full_book"  # 'full_book' or 'selected_text'


class SessionUpdateModeRequest(BaseModel):
    mode: str  # 'full_book' or 'selected_text'


class SessionResponse(BaseModel):
    id: UUID
    created_at: datetime
    mode: str

    class Config:
        from_attributes = True


class SessionModeResponse(BaseModel):
    id: UUID
    mode: str

    class Config:
        from_attributes = True