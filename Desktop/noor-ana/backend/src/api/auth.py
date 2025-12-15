from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional
from ..core.config import settings
from ..core.database import get_async_db
from ..db.models import User
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from passlib.context import CryptContext
import jwt
import uuid
from datetime import datetime, timedelta

router = APIRouter()

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

# Pydantic models for request/response
class SignupRequest(BaseModel):
    email: str
    password: str

class LoginRequest(BaseModel):
    email: str
    password: str

class UserResponse(BaseModel):
    id: str
    email: str
    hardware_profile: Optional[dict] = None
    learning_goals: Optional[list] = None
    accessibility_prefs: Optional[dict] = None
    is_active: bool
    created_at: Optional[str] = None

def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password):
    return pwd_context.hash(password)

async def get_current_user_id(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> str:
    """
    Extract and verify JWT token from Authorization header.
    Returns the user ID from the token.
    """
    token = credentials.credentials

    try:
        payload = jwt.decode(token, settings.BETTER_AUTH_SECRET, algorithms=["HS256"])
        user_id: str = payload.get("sub")

        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid authentication credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        return user_id
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

@router.post("/users/signup", response_model=UserResponse)
async def signup(
    request: SignupRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Create a new user account with password hashing.
    This integrates with our database model and uses proper password hashing.
    """
    # Check if user already exists
    result = await db.execute(select(User).filter(User.email == request.email))
    existing_user = result.scalar_one_or_none()

    if existing_user:
        raise HTTPException(status_code=400, detail="User already exists")

    # Hash the password
    password_hash = get_password_hash(request.password)

    # Create new user
    user = User(
        id=str(uuid.uuid4()),
        email=request.email,
        password_hash=password_hash,
        hardware_profile={},  # Initialize with empty profile
        learning_goals=[],
        accessibility_prefs={},
        is_active=True
    )

    db.add(user)
    await db.commit()
    await db.refresh(user)

    return UserResponse(
        id=user.id,
        email=user.email,
        hardware_profile=user.hardware_profile,
        learning_goals=user.learning_goals,
        accessibility_prefs=user.accessibility_prefs,
        is_active=user.is_active,
        created_at=user.created_at.isoformat() if user.created_at else None
    )


@router.post("/users/login")
async def login(
    request: LoginRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Authenticate user and return session token.
    """
    # Find the user by email
    result = await db.execute(select(User).filter(User.email == request.email))
    user = result.scalar_one_or_none()

    if not user or not verify_password(request.password, user.password_hash):
        raise HTTPException(status_code=400, detail="Invalid credentials")

    # Update last login time
    user.last_login = datetime.now()
    await db.commit()

    # Create JWT token
    token_data = {
        "sub": user.id,
        "email": user.email,
        "exp": datetime.utcnow() + timedelta(days=7)
    }
    token = jwt.encode(token_data, settings.BETTER_AUTH_SECRET, algorithm="HS256")

    return {"access_token": token, "token_type": "bearer"}


@router.get("/users/me", response_model=UserResponse)
async def get_current_user(
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get current user profile with bearer token validation.
    Extracts user ID from JWT token in Authorization header.
    """
    # Get the user from the database
    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return UserResponse(
        id=user.id,
        email=user.email,
        hardware_profile=user.hardware_profile,
        learning_goals=user.learning_goals,
        accessibility_prefs=user.accessibility_prefs,
        is_active=user.is_active,
        created_at=user.created_at.isoformat() if user.created_at else None
    )