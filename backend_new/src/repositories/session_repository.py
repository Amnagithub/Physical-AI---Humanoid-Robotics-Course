from sqlalchemy.orm import Session
from typing import Optional, List
from ..models.database import UserSession


class SessionRepository:
    def __init__(self, db: Session):
        self.db = db

    def create_session(self, mode: str = "full_book") -> UserSession:
        db_session = UserSession(mode=mode)
        self.db.add(db_session)
        self.db.commit()
        self.db.refresh(db_session)
        return db_session

    def get_session(self, session_id: str) -> Optional[UserSession]:
        return self.db.query(UserSession).filter(UserSession.id == session_id).first()

    def update_session_mode(self, session_id: str, mode: str) -> Optional[UserSession]:
        db_session = self.get_session(session_id)
        if db_session:
            db_session.mode = mode
            self.db.commit()
            self.db.refresh(db_session)
        return db_session