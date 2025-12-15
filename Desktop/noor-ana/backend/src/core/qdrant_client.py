from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.async_qdrant_client import AsyncQdrantClient
from ..core.config import settings
import logging

logger = logging.getLogger(__name__)

# Synchronous Qdrant client
sync_qdrant_client = QdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY,
    prefer_grpc=False  # Use HTTP instead of gRPC for better compatibility
)

# Async Qdrant client (for FastAPI endpoints)
async_qdrant_client = AsyncQdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY,
    prefer_grpc=False  # Use HTTP instead of gRPC for better compatibility
)

async def initialize_qdrant_collections():
    """
    Initialize required collections in Qdrant if they don't exist
    Note: Using Gemini embeddings which are 768 dimensions
    """
    try:
        # Check if English content collection exists, create if not
        collections_response = await async_qdrant_client.get_collections()
        collection_names = [collection.name for collection in collections_response.collections]

        if "book_content_en" not in collection_names:
            await async_qdrant_client.create_collection(
                collection_name="book_content_en",
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),  # Gemini: 768 dims
            )
            logger.info("Created Qdrant collection: book_content_en (768 dimensions for Gemini)")

        if "book_content_ur" not in collection_names:
            await async_qdrant_client.create_collection(
                collection_name="book_content_ur",
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),  # Gemini: 768 dims
            )
            logger.info("Created Qdrant collection: book_content_ur (768 dimensions for Gemini)")

        logger.info("Qdrant collections are ready")
    except Exception as e:
        logger.error(f"Error initializing Qdrant collections: {e}")
        raise