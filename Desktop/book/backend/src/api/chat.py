from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
from typing import List, Optional

from ..services.rag import RAGService # Import the RAGService

# Define Pydantic models for API requests and responses
class ChatRequest(BaseModel):
    query: str

class SelectedContextChatRequest(BaseModel):
    query: str
    context: str

class ChatResponse(BaseModel):
    answer: str
    sources: List[str] = []

router = APIRouter()

# Initialize RAGService
rag_service = RAGService()

@router.post("/chat", response_model=ChatResponse)
async def chat_with_full_context(request: ChatRequest):
    """
    Receives a user query and returns an answer based on the entire content of the book.
    """
    try:
        response = rag_service.query(request.query)
        return ChatResponse(answer=response["answer"], sources=response["sources"])
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="An error occurred while processing your request.")


@router.post("/chat-with-selection", response_model=ChatResponse)
async def chat_with_selected_context(request: SelectedContextChatRequest):
    """
    Receives a user query and a specific text snippet, returning an answer based only on that snippet.
    """
    try:
        response = rag_service.query_with_context(request.query, request.context)
        return ChatResponse(answer=response["answer"], sources=response["sources"])
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="An error occurred while processing your request.")
