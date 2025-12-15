from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from ..core.database import get_async_db
from ..db.models import UserProgress
from ..api.auth import get_current_user_id
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime
import uuid

router = APIRouter()

# Pydantic models
class ProgressUpdateRequest(BaseModel):
    chapter_id: str
    completion_percentage: int
    time_spent_seconds: Optional[int] = 0
    exercises_completed: Optional[List[str]] = []
    quiz_scores: Optional[Dict[str, float]] = {}

class ProgressResponse(BaseModel):
    id: str
    user_id: str
    chapter_id: str
    completion_percentage: int
    last_accessed: Optional[str]
    time_spent_seconds: int
    exercises_completed: Optional[List[str]]
    quiz_scores: Optional[Dict[str, float]]

@router.post("/progress", response_model=ProgressResponse)
async def update_progress(
    request: ProgressUpdateRequest,
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Update chapter progress for the current user.
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    # Validate completion percentage
    if not 0 <= request.completion_percentage <= 100:
        raise HTTPException(status_code=400, detail="Completion percentage must be between 0 and 100")

    # Check if a progress record already exists for this user and chapter
    result = await db.execute(
        select(UserProgress)
        .filter(UserProgress.user_id == user_id)
        .filter(UserProgress.chapter_id == request.chapter_id)
    )
    progress = result.scalar_one_or_none()

    if progress:
        # Update existing progress
        progress.completion_percentage = request.completion_percentage
        progress.last_accessed = datetime.now()
        progress.time_spent_seconds = request.time_spent_seconds
        progress.exercises_completed = request.exercises_completed
        progress.quiz_scores = request.quiz_scores
    else:
        # Create new progress record
        progress = UserProgress(
            id=str(uuid.uuid4()),
            user_id=user_id,
            chapter_id=request.chapter_id,
            completion_percentage=request.completion_percentage,
            time_spent_seconds=request.time_spent_seconds,
            exercises_completed=request.exercises_completed,
            quiz_scores=request.quiz_scores
        )
        db.add(progress)

    await db.commit()
    await db.refresh(progress)

    return ProgressResponse(
        id=progress.id,
        user_id=progress.user_id,
        chapter_id=progress.chapter_id,
        completion_percentage=progress.completion_percentage,
        last_accessed=progress.last_accessed.isoformat() if progress.last_accessed else None,
        time_spent_seconds=progress.time_spent_seconds,
        exercises_completed=progress.exercises_completed,
        quiz_scores=progress.quiz_scores
    )

@router.get("/progress", response_model=List[ProgressResponse])
async def get_progress(
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get all progress records for the current user.
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    # Get all progress records for the user
    result = await db.execute(
        select(UserProgress)
        .filter(UserProgress.user_id == user_id)
    )
    progress_records = result.scalars().all()

    return [
        ProgressResponse(
            id=record.id,
            user_id=record.user_id,
            chapter_id=record.chapter_id,
            completion_percentage=record.completion_percentage,
            last_accessed=record.last_accessed.isoformat() if record.last_accessed else None,
            time_spent_seconds=record.time_spent_seconds,
            exercises_completed=record.exercises_completed,
            quiz_scores=record.quiz_scores
        )
        for record in progress_records
    ]