from sqlalchemy.orm import Session
from typing import Optional, List
from uuid import UUID
from ..models.database import Question


class QuestionRepository:
    def __init__(self, db: Session):
        self.db = db

    def create_question(self, session_id: str, content: str, selected_text: Optional[str] = None, context_mode: str = "full_book") -> Question:
        db_question = Question(
            session_id=session_id,
            content=content,
            selected_text=selected_text,
            context_mode=context_mode
        )
        self.db.add(db_question)
        self.db.commit()
        self.db.refresh(db_question)
        return db_question

    def get_question(self, question_id: str) -> Optional[Question]:
        return self.db.query(Question).filter(Question.id == question_id).first()

    def get_questions_by_session(self, session_id: str) -> List[Question]:
        return self.db.query(Question).filter(Question.session_id == session_id).all()