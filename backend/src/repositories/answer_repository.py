from sqlalchemy.orm import Session
from typing import Optional, List
from ..models.database import Answer


class AnswerRepository:
    def __init__(self, db: Session):
        self.db = db

    def create_answer(self, question_id: str, content: str, sources: List[str], confidence_score: Optional[float] = None) -> Answer:
        db_answer = Answer(
            question_id=question_id,
            content=content,
            sources=sources,
            confidence_score=confidence_score
        )
        self.db.add(db_answer)
        self.db.commit()
        self.db.refresh(db_answer)
        return db_answer

    def get_answer(self, answer_id: str) -> Optional[Answer]:
        return self.db.query(Answer).filter(Answer.id == answer_id).first()

    def get_answers_by_question(self, question_id: str) -> Optional[Answer]:
        return self.db.query(Answer).filter(Answer.question_id == question_id).first()