from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class SessionCreate(BaseModel):
    mode: str = "full_book"  # Default to full_book mode


class SessionResponse(BaseModel):
    id: str
    created_at: str
    mode: str
    last_activity: Optional[str] = None

    class Config:
        from_attributes = True