import redis.asyncio as redis
from ..core.config import settings
import logging

logger = logging.getLogger(__name__)

# Async Redis client
redis_client = redis.from_url(
    settings.REDIS_URL,
    decode_responses=True,
    encoding="utf-8"
)

async def get_redis_client():
    """
    Dependency to provide Redis client
    """
    return redis_client

async def initialize_redis():
    """
    Test Redis connection and initialize
    """
    try:
        # Test connection
        await redis_client.ping()
        logger.info("Connected to Redis successfully")
    except Exception as e:
        logger.error(f"Failed to connect to Redis: {e}")
        raise