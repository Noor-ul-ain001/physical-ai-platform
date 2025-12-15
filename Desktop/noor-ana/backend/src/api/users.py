from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from ..core.config import settings
from ..core.database import get_async_db
from ..db.models import User
from ..api.auth import get_current_user_id
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
import uuid

router = APIRouter()

# Pydantic models
class HardwareProfile(BaseModel):
    gpu: Optional[str] = None
    robotics_kit: Optional[str] = None
    experience_level: Optional[str] = None

class ProfileUpdateRequest(BaseModel):
    hardware_profile: Optional[HardwareProfile] = None
    learning_goals: Optional[List[str]] = None
    accessibility_prefs: Optional[Dict[str, Any]] = None

class UserResponse(BaseModel):
    id: str
    email: str
    hardware_profile: Optional[Dict[str, Any]]
    learning_goals: Optional[List[str]]
    accessibility_prefs: Optional[Dict[str, Any]]
    created_at: Optional[str]
    is_active: bool

@router.patch("/users/me", response_model=UserResponse)
async def update_user_profile(
    request: ProfileUpdateRequest,
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Update user profile information.
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    # Get the user from the database
    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    # Update the profile fields if provided
    if request.hardware_profile is not None:
        user.hardware_profile = request.hardware_profile.dict()

    if request.learning_goals is not None:
        user.learning_goals = request.learning_goals

    if request.accessibility_prefs is not None:
        user.accessibility_prefs = request.accessibility_prefs

    # Commit the changes
    await db.commit()
    await db.refresh(user)

    return UserResponse(
        id=user.id,
        email=user.email,
        hardware_profile=user.hardware_profile,
        learning_goals=user.learning_goals,
        accessibility_prefs=user.accessibility_prefs,
        created_at=user.created_at.isoformat() if user.created_at else None,
        is_active=user.is_active
    )