from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    # Database
    DATABASE_URL: str
    
    # Qdrant
    QDRANT_URL: str
    QDRANT_API_KEY: str

    # Google Gemini (Free API)
    GEMINI_API_KEY: str

    # Groq API
    GROQ_API_KEY: str

    # Redis
    REDIS_URL: str
    
    # Google Translate
    GOOGLE_TRANSLATE_API_KEY: str
    
    # Better Auth
    BETTER_AUTH_SECRET: str
    BETTER_AUTH_URL: str
    
    # CORS
    ALLOWED_ORIGINS: List[str] = ["*"]  # In production, replace with specific origins
    
    # Other settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Physical AI & Humanoid Robotics Platform"
    
    class Config:
        env_file = ".env"


settings = Settings()