"""
RAG (Retrieval-Augmented Generation) service for the Physical AI & Humanoid Robotics platform.
Implements semantic search and response generation using Google Gemini and Qdrant.
"""

from typing import List, Dict, Any, Optional
from src.core.qdrant_client import async_qdrant_client
from src.core.embeddings import get_single_embedding
from src.core.config import settings
from src.core.redis_client import get_redis_client
from groq import Groq
import logging
import json
from dataclasses import dataclass
import time
import asyncio

logger = logging.getLogger(__name__)

# Configure Groq client
groq_client = Groq(api_key=settings.GROQ_API_KEY)

@dataclass
class Citation:
    source: str
    excerpt: str
    similarity: float

@dataclass
class RAGResponse:
    answer: str
    citations: List[Citation]
    confidence: float
    retrieval_time_ms: int
    retrieval_context: Dict[str, Any]

class RAGService:
    def __init__(self):
        # Use Groq with llama model
        self.model_name = "llama-3.3-70b-versatile"  # Fast and capable Groq model
        self.collection_name = "book_content_en"

    async def _get_cache_key(self, query: str, context_mode: str) -> str:
        """
        Generate a cache key for the given query and context mode.
        """
        import hashlib
        query_hash = hashlib.md5(query.encode()).hexdigest()
        return f"rag_response:{context_mode}:{query_hash}"

    async def _get_cached_response(self, cache_key: str) -> Optional[RAGResponse]:
        """
        Retrieve a cached response from Redis.
        """
        redis_client = await get_redis_client()
        cached_data = await redis_client.get(cache_key)

        if cached_data:
            data = json.loads(cached_data)
            # Reconstruct the RAGResponse object
            return RAGResponse(
                answer=data['answer'],
                citations=[Citation(**c) for c in data['citations']],
                confidence=data['confidence'],
                retrieval_time_ms=data['retrieval_time_ms'],
                retrieval_context=data['retrieval_context']
            )
        return None

    async def _cache_response(self, cache_key: str, response: RAGResponse, ttl: int = 3600):
        """
        Cache the response in Redis.
        """
        redis_client = await get_redis_client()
        data = {
            "answer": response.answer,
            "citations": [{"source": c.source, "excerpt": c.excerpt, "similarity": c.similarity}
                          for c in response.citations],
            "confidence": response.confidence,
            "retrieval_time_ms": response.retrieval_time_ms,
            "retrieval_context": response.retrieval_context
        }
        await redis_client.setex(cache_key, ttl, json.dumps(data))

    async def retrieve_relevant_chunks(
        self,
        query: str,
        top_k: int = 5,
        threshold: float = 0.3
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content chunks from Qdrant based on the query.
        """
        start_time = time.time()

        try:
            # Generate embedding for the query
            query_embedding = await get_single_embedding(query)

            # Perform semantic search in Qdrant
            search_results = await async_qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold,
                with_payload=True
            )

            # Format results
            retrieved_chunks = []
            for result in search_results:
                retrieved_chunks.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "chapter_id": result.payload.get("chapter_id", ""),
                    "section_id": result.payload.get("section_id", ""),
                    "difficulty_level": result.payload.get("difficulty_level", ""),
                    "hardware_requirement": result.payload.get("hardware_requirement", ""),
                    "similarity": result.score
                })

            retrieval_time_ms = int((time.time() - start_time) * 1000)
            logger.info(f"Retrieved {len(retrieved_chunks)} chunks in {retrieval_time_ms}ms for query: {query[:50]}...")

            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error during retrieval: {str(e)}")
            raise

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[Dict[str, Any]],
        context_mode: str = "full_book"
    ) -> RAGResponse:
        """
        Generate a response using OpenAI based on retrieved content.
        """
        start_time = time.time()

        try:
            # Combine retrieved chunks into context
            context_parts = []
            citations = []
            max_context_length = 3000  # Limit to prevent exceeding token limits

            current_length = 0
            for chunk in retrieved_chunks:
                chunk_text = chunk["content"]

                # Skip if adding this chunk would exceed the limit
                if current_length + len(chunk_text) > max_context_length:
                    break

                context_parts.append(chunk_text)
                current_length += len(chunk_text)

                # Add citation
                citations.append(Citation(
                    source=chunk["chapter_id"],
                    excerpt=chunk_text[:100] + "..." if len(chunk_text) > 100 else chunk_text,
                    similarity=chunk["similarity"]
                ))

            context = "\n\n".join(context_parts)

            # Create prompt combining system instructions and user query
            prompt = f"""You are an expert teaching assistant for the Physical AI & Humanoid Robotics educational platform.
            Provide accurate, helpful answers based on the provided content from the curriculum.
            Always be specific, cite sources where possible, and maintain a professional teaching tone.
            If the answer is not in the provided context, clearly state that you don't have that information.

            Context from the curriculum:
            {context}

            Question: {query}

            Please provide a clear, concise answer based on the context above."""

            # Call Groq API (using asyncio.to_thread for synchronous API)
            response = await asyncio.to_thread(
                groq_client.chat.completions.create,
                model=self.model_name,
                messages=[
                    {
                        "role": "system",
                        "content": "You are an expert teaching assistant for the Physical AI & Humanoid Robotics educational platform. Provide accurate, helpful answers based on the provided content from the curriculum."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.3,  # Lower temperature for more consistent responses
                max_tokens=500,  # Limit response length
            )

            answer = response.choices[0].message.content

            # Calculate confidence based on similarity scores
            if retrieved_chunks:
                avg_similarity = sum(chunk["similarity"] for chunk in retrieved_chunks) / len(retrieved_chunks)
                confidence = min(1.0, avg_similarity * 2)  # Scale similarity to 0-1 range
            else:
                confidence = 0.0

            generation_time_ms = int((time.time() - start_time) * 1000)

            return RAGResponse(
                answer=answer,
                citations=citations,
                confidence=confidence,
                retrieval_time_ms=generation_time_ms,
                retrieval_context={
                    "chunks_retrieved": len(retrieved_chunks),
                    "top_k": len(retrieved_chunks),
                    "threshold": 0.3
                }
            )

        except Exception as e:
            logger.error(f"Error during response generation: {str(e)}")
            raise

    async def _generate_fallback_response(self, query: str) -> RAGResponse:
        """
        Generate a response without Qdrant retrieval (fallback mode).
        Uses Groq to answer directly based on its training data.
        """
        start_time = time.time()

        try:
            prompt = f"""You are an expert teaching assistant for a Physical AI & Humanoid Robotics educational platform.
            The student has asked: {query}

            Please provide a helpful, educational response about Physical AI, Humanoid Robotics, ROS 2, or related topics.
            If you're not sure about the answer, be honest and suggest general resources or approaches.

            Note: This is a fallback response as the full curriculum database is currently unavailable."""

            # Call Groq API
            response = await asyncio.to_thread(
                groq_client.chat.completions.create,
                model=self.model_name,
                messages=[
                    {
                        "role": "system",
                        "content": "You are an expert teaching assistant for a Physical AI & Humanoid Robotics educational platform."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.7,
                max_tokens=500,
            )

            answer = response.choices[0].message.content + "\n\n_Note: This response is generated without access to the full curriculum. For detailed, course-specific information, please ensure the vector database is properly configured._"

            generation_time_ms = int((time.time() - start_time) * 1000)

            return RAGResponse(
                answer=answer,
                citations=[],
                confidence=0.5,  # Lower confidence for fallback
                retrieval_time_ms=generation_time_ms,
                retrieval_context={
                    "chunks_retrieved": 0,
                    "fallback_mode": True
                }
            )
        except Exception as e:
            logger.error(f"Fallback generation also failed: {e}")
            # Return a basic error response
            return RAGResponse(
                answer="I apologize, but I'm currently unable to process your question due to a technical issue. Please try again later or contact support if the problem persists.",
                citations=[],
                confidence=0.0,
                retrieval_time_ms=0,
                retrieval_context={"error": True}
            )

    async def get_full_context_response(
        self,
        query: str,
        top_k: int = 5,
        threshold: float = 0.3
    ) -> RAGResponse:
        """
        Get a response using full-book context.
        Check cache first, otherwise generate and cache the response.
        """
        cache_key = await self._get_cache_key(query, "full_book")

        # Try to get cached response
        try:
            cached_response = await self._get_cached_response(cache_key)
            if cached_response:
                logger.info(f"Cache hit for query: {query[:50]}...")
                return cached_response
        except Exception as e:
            logger.warning(f"Cache retrieval failed: {e}")

        # Generate new response
        try:
            retrieved_chunks = await self.retrieve_relevant_chunks(query, top_k, threshold)
            response = await self.generate_response(query, retrieved_chunks, context_mode="full_book")

            # Try to cache the response
            try:
                await self._cache_response(cache_key, response)
            except Exception as e:
                logger.warning(f"Cache storage failed: {e}")

            return response
        except Exception as e:
            # Fallback: Generate response without Qdrant retrieval
            logger.warning(f"Retrieval failed, using fallback mode: {e}")
            return await self._generate_fallback_response(query)

    async def get_selective_context_response(
        self,
        query: str,
        selected_text: str,
        page_context: Optional[str] = None,
        top_k: int = 3
    ) -> RAGResponse:
        """
        Get a response using selective context (from selected text only).
        Check cache first, otherwise generate and cache the response.
        """
        cache_key = await self._get_cache_key(f"{query}::{selected_text[:50]}", "selective")

        # Try to get cached response
        cached_response = await self._get_cached_response(cache_key)
        if cached_response:
            logger.info(f"Cache hit for selective query: {query[:50]}...")
            return cached_response

        # For selective context, we might use the selected text directly
        # or use it to refine the query and retrieve more targeted results
        refined_query = f"{query} based on: {selected_text[:500]}..."  # Limit context length

        retrieved_chunks = await self.retrieve_relevant_chunks(refined_query, top_k, threshold=0.1)

        # Add the selected text as an additional context
        if selected_text:
            # Insert selected text at the beginning for higher importance
            retrieved_chunks.insert(0, {
                "id": "selected_text",
                "content": selected_text,
                "chapter_id": "selected",
                "section_id": "selected",
                "difficulty_level": "n/a",
                "hardware_requirement": "n/a",
                "similarity": 1.0  # Highest priority
            })

        response = await self.generate_response(query, retrieved_chunks, context_mode="selective")

        # Mark that response was based on selection
        response.retrieval_context["sourced_from_selection"] = True

        # Cache the response
        await self._cache_response(cache_key, response)

        return response

    async def check_hallucination(self, answer: str, retrieved_chunks: List[Dict[str, Any]]) -> bool:
        """
        Check if the generated answer contains hallucinations by verifying
        that claims in the answer are supported by retrieved chunks.
        This is a simplified implementation - a full system would use more sophisticated techniques.
        """
        answer_lower = answer.lower()

        # Check if answer references content from retrieved chunks
        has_supporting_evidence = False
        for chunk in retrieved_chunks:
            chunk_content_lower = chunk["content"].lower()
            # Simple check: if answer contains terms from chunks, it's likely not hallucinated
            if any(word in answer_lower for word in chunk_content_lower.split()[:20]):  # Check first 20 words
                has_supporting_evidence = True
                break

        return not has_supporting_evidence  # Returns True if hallucination detected