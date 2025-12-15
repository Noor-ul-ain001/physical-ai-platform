from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .core.config import settings

app = FastAPI(
    title="Physical AI & Humanoid Robotics Platform API",
    version="1.0.0",
    description="Backend API for educational platform with RAG chatbot, personalization, and translation"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics Platform API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# Include API routes
from .api import chat, users, content, bookmarks, progress, auth
app.include_router(chat.router, prefix="/api")
app.include_router(users.router, prefix="/api")
app.include_router(content.router, prefix="/api")
app.include_router(bookmarks.router, prefix="/api")
app.include_router(progress.router, prefix="/api")
app.include_router(auth.router, prefix="/api")

# Initialize services on startup
@app.on_event("startup")
async def startup_event():
    from .core.qdrant_client import initialize_qdrant_collections
    from .core.redis_client import initialize_redis
    from .core.database import async_engine
    from .db.models import Base
    import logging

    logger = logging.getLogger(__name__)

    # Create database tables if they don't exist
    try:
        async with async_engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
        logger.info("Database tables created successfully")
    except Exception as e:
        logger.warning(f"Database initialization failed: {e}. App will run without database.")

    # Initialize Qdrant collections (optional - app can run without it)
    try:
        await initialize_qdrant_collections()
    except Exception as e:
        logger.warning(f"Qdrant initialization failed: {e}. App will run without vector search capabilities.")

    # Initialize Redis (optional - app can run without caching)
    try:
        await initialize_redis()
    except Exception as e:
        logger.warning(f"Redis initialization failed: {e}. App will run without caching.")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)