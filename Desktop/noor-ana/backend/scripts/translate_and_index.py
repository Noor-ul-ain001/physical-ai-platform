#!/usr/bin/env python3
"""
Script to translate and index content to Urdu for the Physical AI & Humanoid Robotics platform.
This script translates educational content to Urdu and indexes it into a separate Qdrant collection.
"""

import os
import asyncio
from pathlib import Path
from typing import List, Dict, Any

from src.core.qdrant_client import async_qdrant_client, initialize_qdrant_collections
from src.core.embeddings import get_embeddings
from src.services.translation import TranslationService
from src.core.config import settings


async def translate_and_index_to_urdu(
    source_collection: str = "book_content_en",
    target_collection: str = "book_content_ur",
    content_dir: str = "../frontend/docs"
) -> None:
    """
    Translate content from English collection to Urdu and index it in a new collection.
    """
    print(f"Starting translation and indexing from '{source_collection}' to '{target_collection}'...")
    
    # Initialize translation service
    translation_service = TranslationService(api_key=settings.GOOGLE_TRANSLATE_API_KEY)
    
    # Get all content from the English collection
    print("Fetching content from English collection...")
    try:
        records = await async_qdrant_client.scroll(
            collection_name=source_collection,
            limit=10000,  # Adjust as needed
            with_payload=True,
            with_vectors=False
        )
        
        all_points = records[0]
        print(f"Fetched {len(all_points)} records from '{source_collection}'")
    except Exception as e:
        print(f"Error fetching records from {source_collection}: {str(e)}")
        return
    
    # Process each record
    translated_points = []
    
    for i, point in enumerate(all_points):
        try:
            payload = point.payload
            content = payload.get("content", "")
            
            print(f"Translating record {i+1}/{len(all_points)}: {payload.get('chapter_id', 'Unknown')}")
            
            # Translate the content
            translation_result = await translation_service.translate_content(
                content, target_language="ur"
            )
            
            # Create new payload with translated content
            translated_payload = {
                **payload,  # Copy all original payload fields
                "content": translation_result.translated_content,
                "content_original": content,  # Keep original for reference
                "language": "ur"
            }
            
            # Get embedding for the translated content
            embedding = await get_embeddings([translation_result.translated_content])
            
            # Create new point for the Urdu collection
            translated_point = {
                "id": point.id,  # Use same ID as original
                "vector": embedding[0],
                "payload": translated_payload
            }
            
            translated_points.append(translated_point)
            
            # Process in batches to avoid overwhelming the API
            if len(translated_points) >= 20:  # Batch size
                print(f"Uploading batch of {len(translated_points)} points to '{target_collection}'...")
                
                # Clear the target collection if this is the first batch
                if i < 20:
                    try:
                        await async_qdrant_client.delete_collection(target_collection)
                        print(f"Deleted existing '{target_collection}' collection")
                    except:
                        pass  # Collection might not exist yet, which is fine
                
                # Create collection if it doesn't exist
                await async_qdrant_client.create_collection(
                    collection_name=target_collection,
                    vectors_config={"size": 1536, "distance": "Cosine"},  # Same as English collection
                )
                
                # Upload batch
                await async_qdrant_client.upload_points(
                    collection_name=target_collection,
                    points=translated_points
                )
                
                print(f"Uploaded batch to '{target_collection}'")
                translated_points = []  # Reset for next batch
            
        except Exception as e:
            print(f"Error processing record {i+1}: {str(e)}")
            continue
    
    # Upload remaining points
    if translated_points:
        print(f"Uploading final batch of {len(translated_points)} points to '{target_collection}'...")
        
        # Create collection if it doesn't exist
        await async_qdrant_client.create_collection(
            collection_name=target_collection,
            vectors_config={"size": 1536, "distance": "Cosine"},
        )
        
        # Upload remaining points
        await async_qdrant_client.upload_points(
            collection_name=target_collection,
            points=translated_points
        )
        
        print(f"Uploaded final batch to '{target_collection}'")
    
    print(f"Translation and indexing completed. Total records processed: {len(all_points)}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Translate and index content to Urdu")
    parser.add_argument("--source", default="book_content_en", help="Source collection name")
    parser.add_argument("--target", default="book_content_ur", help="Target collection name")
    parser.add_argument("--content-dir", default="../frontend/docs", help="Source content directory")
    
    args = parser.parse_args()
    
    # Run async function
    asyncio.run(translate_and_index_to_urdu(
        source_collection=args.source,
        target_collection=args.target,
        content_dir=args.content_dir
    ))


if __name__ == "__main__":
    main()