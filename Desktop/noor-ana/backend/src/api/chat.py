from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
from ..services.rag import RAGService, RAGResponse
from ..agents.chat_agent import ChatAgent
import uuid

router = APIRouter()

# Pydantic models for request/response
class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    top_k: Optional[int] = 5


class SelectiveChatRequest(BaseModel):
    query: str
    selected_text: str
    page_context: Optional[str] = None
    session_id: Optional[str] = None
    top_k: Optional[int] = 3


class CitationResponse(BaseModel):
    source: str
    excerpt: str
    similarity: float


class ChatResponse(BaseModel):
    answer: str
    citations: List[CitationResponse]
    confidence: float
    retrieval_time_ms: int


class SelectiveChatResponse(BaseModel):
    answer: str
    mode: str
    sourced_from_selection: bool
    insufficient_context: bool
    suggestions: List[str]


# Initialize RAG service
rag_service = RAGService()


@router.post("/chat", response_model=ChatResponse)
async def chat_full_context(request: ChatRequest):
    """
    Chat endpoint with full-book context retrieval.
    """
    try:
        response = await rag_service.get_full_context_response(
            query=request.query,
            top_k=request.top_k
        )

        return ChatResponse(
            answer=response.answer,
            citations=[
                CitationResponse(
                    source=citation.source,
                    excerpt=citation.excerpt,
                    similarity=citation.similarity
                )
                for citation in response.citations
            ],
            confidence=response.confidence,
            retrieval_time_ms=response.retrieval_time_ms
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@router.post("/chat/selective", response_model=ChatResponse)
async def chat_selective_context(request: SelectiveChatRequest):
    """
    Chat endpoint with selective context (selected text only).
    """
    try:
        response = await rag_service.get_selective_context_response(
            query=request.query,
            selected_text=request.selected_text,
            page_context=request.page_context,
            top_k=request.top_k
        )

        return ChatResponse(
            answer=response.answer,
            citations=[
                CitationResponse(
                    source=citation.source,
                    excerpt=citation.excerpt,
                    similarity=citation.similarity
                )
                for citation in response.citations
            ],
            confidence=response.confidence,
            retrieval_time_ms=response.retrieval_time_ms
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing selective chat request: {str(e)}")