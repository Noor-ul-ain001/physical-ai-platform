from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from ..core.database import get_async_db
from ..db.models import Bookmark
from ..api.auth import get_current_user_id
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime
import uuid

router = APIRouter()

# Pydantic models
class BookmarkCreateRequest(BaseModel):
    chapter_id: str
    section_id: Optional[str] = None
    notes: Optional[str] = None

class BookmarkUpdateRequest(BaseModel):
    notes: Optional[str] = None

class BookmarkResponse(BaseModel):
    id: str
    user_id: str
    chapter_id: str
    section_id: Optional[str]
    notes: Optional[str]
    created_at: Optional[str]

@router.post("/bookmarks", response_model=BookmarkResponse)
async def create_bookmark(
    request: BookmarkCreateRequest,
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Create a new bookmark for the current user.
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    # Check if bookmark already exists for this user, chapter, and section
    result = await db.execute(
        select(Bookmark)
        .filter(Bookmark.user_id == user_id)
        .filter(Bookmark.chapter_id == request.chapter_id)
        .filter(Bookmark.section_id == (request.section_id or ""))
    )
    existing_bookmark = result.scalar_one_or_none()

    if existing_bookmark:
        raise HTTPException(status_code=400, detail="Bookmark already exists for this section")

    # Create new bookmark
    bookmark = Bookmark(
        id=str(uuid.uuid4()),
        user_id=user_id,
        chapter_id=request.chapter_id,
        section_id=request.section_id,
        notes=request.notes
    )
    db.add(bookmark)
    await db.commit()
    await db.refresh(bookmark)

    return BookmarkResponse(
        id=bookmark.id,
        user_id=bookmark.user_id,
        chapter_id=bookmark.chapter_id,
        section_id=bookmark.section_id,
        notes=bookmark.notes,
        created_at=bookmark.created_at.isoformat() if bookmark.created_at else None
    )

@router.get("/bookmarks", response_model=List[BookmarkResponse])
async def get_bookmarks(
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get all bookmarks for the current user.
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    # Get all bookmarks for the user
    result = await db.execute(
        select(Bookmark)
        .filter(Bookmark.user_id == user_id)
    )
    bookmarks = result.scalars().all()

    return [
        BookmarkResponse(
            id=bookmark.id,
            user_id=bookmark.user_id,
            chapter_id=bookmark.chapter_id,
            section_id=bookmark.section_id,
            notes=bookmark.notes,
            created_at=bookmark.created_at.isoformat() if bookmark.created_at else None
        )
        for bookmark in bookmarks
    ]

@router.delete("/bookmarks/{bookmark_id}")
async def delete_bookmark(
    bookmark_id: str,
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Delete a specific bookmark for the current user.
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    # Find the bookmark
    result = await db.execute(
        select(Bookmark)
        .filter(Bookmark.id == bookmark_id)
        .filter(Bookmark.user_id == user_id)
    )
    bookmark = result.scalar_one_or_none()

    if not bookmark:
        raise HTTPException(status_code=404, detail="Bookmark not found")

    # Delete the bookmark
    await db.delete(bookmark)
    await db.commit()

    return {"message": "Bookmark deleted successfully"}