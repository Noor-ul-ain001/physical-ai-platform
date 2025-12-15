#!/usr/bin/env python3
"""
Content indexing script for the Physical AI & Humanoid Robotics educational platform.
This script processes educational content from the frontend/docs directory and
indexes it into Qdrant for semantic search capabilities.
"""

import os
import re
from typing import List, Dict, Any
import asyncio
from pathlib import Path

from src.core.qdrant_client import async_qdrant_client
from src.core.embeddings import get_embeddings
from src.core.config import settings


def extract_text_from_mdx(file_path: Path) -> str:
    """
    Extract text content from an MDX file, preserving important content
    while removing frontmatter and code blocks.
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    
    # Remove frontmatter (content between --- delimiters)
    frontmatter_pattern = r'^---\n.*?\n---\n'
    content = re.sub(frontmatter_pattern, '', content, flags=re.DOTALL)
    
    # Extract text while preserving the overall structure
    # Remove code blocks but keep the text around them
    code_block_pattern = r'```.*?\n.*?\n```'
    content = re.sub(code_block_pattern, ' [CODE_BLOCK] ', content, flags=re.DOTALL)
    
    # Remove inline code
    inline_code_pattern = r'`(.*?)`'
    content = re.sub(inline_code_pattern, r' \1 ', content)
    
    # Clean up extra whitespace
    content = re.sub(r'\s+', ' ', content)
    
    return content.strip()


def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Split text into overlapping chunks of specified size.
    """
    words = text.split()
    chunks = []
    
    for i in range(0, len(words), chunk_size - overlap):
        chunk = ' '.join(words[i:i + chunk_size])
        chunks.append(chunk)
        
        # If the last chunk is smaller than overlap, we're done
        if len(words) - i <= chunk_size:
            break
    
    return chunks


async def index_content_to_qdrant(
    content_dir: str = "../frontend/docs",
    collection_name: str = "book_content_en"
) -> None:
    """
    Index content from the specified directory into Qdrant.
    """
    content_path = Path(content_dir)
    
    if not content_path.exists():
        print(f"Content directory {content_path} does not exist.")
        return
    
    # Get all MDX files
    mdx_files = list(content_path.rglob("*.mdx")) + list(content_path.rglob("*.md"))
    
    all_chunks = []
    
    for file_path in mdx_files:
        print(f"Processing {file_path}")
        
        # Extract content
        content = extract_text_from_mdx(file_path)
        
        # Determine difficulty and hardware requirements from path
        # This is a simplified approach - in a real system, this would be more sophisticated
        difficulty = "beginner"  # Default
        if "advanced" in str(file_path).lower():
            difficulty = "advanced"
        elif "intermediate" in str(file_path).lower():
            difficulty = "intermediate"
        
        hardware_requirement = "none"  # Default
        if "jetson" in content.lower() or "orin" in content.lower():
            hardware_requirement = "jetson"
        elif "robot" in content.lower() or "unitree" in content.lower():
            hardware_requirement = "full_robot"
        
        # Chunk the content
        chunks = chunk_text(content)
        
        for idx, chunk in enumerate(chunks):
            # Create document for indexing
            chapter_id = str(file_path.relative_to(content_path).with_suffix(''))
            document = {
                "chapter_id": chapter_id,
                "section_id": f"{chapter_id}-chunk-{idx}",
                "content": chunk,
                "content_type": "text",
                "difficulty_level": difficulty,
                "hardware_requirement": hardware_requirement,
                "chunk_index": idx,
                "total_chunks": len(chunks)
            }
            
            all_chunks.append({
                "document": document,
                "text": chunk
            })
    
    # Generate embeddings for all texts
    print(f"Generating embeddings for {len(all_chunks)} chunks...")
    texts = [item["text"] for item in all_chunks]
    
    # Process in batches to avoid API limits
    batch_size = 20
    all_embeddings = []
    
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i + batch_size]
        batch_embeddings = await get_embeddings(batch_texts)
        all_embeddings.extend(batch_embeddings)
    
    # Prepare points for Qdrant
    points = []
    for idx, (item, embedding) in enumerate(zip(all_chunks, all_embeddings)):
        points.append({
            "id": idx,
            "vector": embedding,
            "payload": item["document"]
        })
    
    # Upload to Qdrant
    print(f"Uploading {len(points)} points to Qdrant collection '{collection_name}'...")
    
    # Clear existing collection if it exists
    try:
        await async_qdrant_client.delete_collection(collection_name)
    except:
        pass  # Collection might not exist yet, which is fine
    
    # Recreate collection
    await async_qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config={"size": 768, "distance": "Cosine"},  # Gemini text-embedding-004 dimensions
    )
    
    # Upload points
    await async_qdrant_client.upload_points(
        collection_name=collection_name,
        points=points
    )
    
    print(f"Successfully indexed {len(points)} chunks from {len(mdx_files)} files into '{collection_name}'")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Index educational content to Qdrant")
    parser.add_argument("--source", default="../frontend/docs", help="Source directory for content")
    parser.add_argument("--collection", default="book_content_en", help="Qdrant collection name")
    
    args = parser.parse_args()
    
    # Run async function
    asyncio.run(index_content_to_qdrant(args.source, args.collection))


if __name__ == "__main__":
    main()