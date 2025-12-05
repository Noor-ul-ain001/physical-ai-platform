import os
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http.models import Filter, FieldCondition, MatchValue
from typing import List, Dict

# Assuming embedding model and collection name are consistent with embed.py
from .core.qdrant_client import get_qdrant_client # This will load .env
from .core.embed import EMBEDDING_MODEL, COLLECTION_NAME


class RAGService:
    def __init__(self):
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.qdrant_client = get_qdrant_client()
        self.embedding_model = EMBEDDING_MODEL
        self.collection_name = COLLECTION_NAME
        self.llm_model = "gpt-3.5-turbo" # Can be configured via env var

    def _get_query_embedding(self, text: str) -> List[float]:
        """Generates an embedding for the given text."""
        response = self.openai_client.embeddings.create(input=text, model=self.embedding_model)
        return response.data[0].embedding

    def _retrieve_context(self, query_embedding: List[float], limit: int = 5) -> List[Dict]:
        """Retrieves relevant text chunks from Qdrant."""
        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            query_filter=None, # No specific filter for full book context
        )
        return [{"text": hit.payload["text"], "source": hit.payload["source"]} for hit in search_result]

    def _generate_answer(self, user_query: str, context: List[Dict]) -> str:
        """Generates an answer using the LLM based on retrieved context."""
        context_str = "\n\n".join([f"Source: {c['source']}\nText: {c['text']}" for c in context])

        prompt = (
            f"You are an AI assistant designed to answer questions about 'The Book of AI-Driven Development'. "
            f"Use ONLY the following context to answer the user's question. If the answer is not found in the context, "
            f"state that you cannot answer the question based on the provided information.\n\n"
            f"Context:\n{context_str}\n\n"
            f"User Question: {user_query}\n"
            f"Answer:"
        )

        response = self.openai_client.chat.completions.create(
            model=self.llm_model,
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7,
            max_tokens=500
        )
        return response.choices[0].message.content

    def query(self, user_query: str) -> Dict:
        """Performs the full RAG pipeline for a user query."""
        query_embedding = self._get_query_embedding(user_query)
        context = self._retrieve_context(query_embedding)
        answer = self._generate_answer(user_query, context)
        
        sources = list(set([c["source"] for c in context])) # Unique sources
        
        return {"answer": answer, "sources": sources}

    def query_with_context(self, user_query: str, provided_context: str) -> Dict:
        """
        Generates an answer using the LLM based *only* on the provided context string.
        This bypasses Qdrant retrieval.
        """
        context_str = f"Provided Context:\n{provided_context}\n\n"

        prompt = (
            f"You are an AI assistant designed to answer questions about a specific text snippet. "
            f"Use ONLY the following context to answer the user's question. If the answer is not found in the context, "
            f"state that you cannot answer the question based on the provided information.\n\n"
            f"Context:\n{provided_context}\n\n"
            f"User Question: {user_query}\n"
            f"Answer:"
        )
        
        response = self.openai_client.chat.completions.create(
            model=self.llm_model,
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7,
            max_tokens=500
        )
        return {"answer": response.choices[0].message.content, "sources": ["User Provided Context"]}


# Example usage (for testing, not part of actual service flow)
if __name__ == '__main__':
    rag_service = RAGService()
    # Ensure you have run embed.py first to populate Qdrant
    test_query = "What is spec-driven development?"
    response = rag_service.query(test_query)
    print(f"\nQuery: {test_query}")
    print(f"Answer: {response['answer']}")
    print(f"Sources: {response['sources']}")
    
    test_context = "Spec-driven development is a powerful methodology that emphasizes clarity and precision before implementation. By defining user stories, requirements, and success criteria upfront, teams can avoid ambiguity and build the right product from the start."
    test_query_with_context = "What does this passage say about methodology?"
    response_with_context = rag_service.query_with_context(test_query_with_context, test_context)
    print(f"\nQuery with context: {test_query_with_context}")
    print(f"Context: {test_context[:50]}...")
    print(f"Answer: {response_with_context['answer']}")
    print(f"Sources: {response_with_context['sources']}")
