import google.generativeai as genai
from ..core.config import settings
import logging
from typing import List
import asyncio

logger = logging.getLogger(__name__)

# Configure Gemini API
genai.configure(api_key=settings.GEMINI_API_KEY)

async def get_embeddings(texts: List[str], model: str = "models/text-embedding-004") -> List[List[float]]:
    """
    Get embeddings for a list of texts using Google's Gemini text-embedding-004 model
    Note: Gemini embeddings are 768 dimensions (vs OpenAI's 1536)
    """
    try:
        # Gemini doesn't support batch embeddings in async, so we do them individually
        embeddings = []
        for text in texts:
            result = await asyncio.to_thread(
                genai.embed_content,
                model=model,
                content=text,
                task_type="retrieval_document"
            )
            embeddings.append(result['embedding'])

        return embeddings
    except Exception as e:
        logger.error(f"Error generating embeddings with Gemini: {e}")
        raise

async def get_single_embedding(text: str, model: str = "models/text-embedding-004") -> List[float]:
    """
    Get embedding for a single text using Gemini
    """
    try:
        result = await asyncio.to_thread(
            genai.embed_content,
            model=model,
            content=text,
            task_type="retrieval_query"  # Use query type for search queries
        )
        return result['embedding']
    except Exception as e:
        logger.error(f"Error generating single embedding with Gemini: {e}")
        raise