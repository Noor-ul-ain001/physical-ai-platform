#!/usr/bin/env python3
"""
Script to verify content indexing in Qdrant.
Checks that content has been properly indexed with expected metadata.
"""

import asyncio
from src.core.qdrant_client import async_qdrant_client


async def check_qdrant_indexing(collection_name: str = "book_content_en"):
    """
    Verify that content has been properly indexed in Qdrant.
    """
    try:
        # Get collection info
        collection_info = await async_qdrant_client.get_collection(collection_name)
        print(f"Collection '{collection_name}' exists with {collection_info.points_count} points")
        
        if collection_info.points_count == 0:
            print("WARNING: Collection is empty!")
            return False
        
        # Sample some points to verify metadata
        limit = min(5, collection_info.points_count)
        points = await async_qdrant_client.scroll(
            collection_name=collection_name,
            limit=limit,
            with_payload=True,
            with_vectors=False
        )
        
        print(f"\nSample of {limit} points from collection:")
        for i, (point, _) in enumerate(points[0]):
            payload = point.payload
            print(f"\nPoint {i+1}:")
            print(f"  Chapter ID: {payload.get('chapter_id', 'N/A')}")
            print(f"  Section ID: {payload.get('section_id', 'N/A')}")
            print(f"  Content Type: {payload.get('content_type', 'N/A')}")
            print(f"  Difficulty: {payload.get('difficulty_level', 'N/A')}")
            print(f"  Hardware Requirement: {payload.get('hardware_requirement', 'N/A')}")
            print(f"  Chunk Index: {payload.get('chunk_index', 'N/A')} of {payload.get('total_chunks', 'N/A')}")
            print(f"  Content Preview: {payload.get('content', '')[:100]}...")
        
        # Verify specific expected content exists
        print("\nChecking for expected content...")
        
        # Search for a common term that should exist in the content
        search_results = await async_qdrant_client.search(
            collection_name=collection_name,
            query_text="ROS 2",  # Common term from the curriculum
            limit=3
        )
        
        if len(search_results) > 0:
            print(f"✓ Found {len(search_results)} results for 'ROS 2'")
            for i, result in enumerate(search_results):
                print(f"  Result {i+1}: {result.payload.get('chapter_id', 'N/A')}")
        else:
            print("⚠ Could not find expected content for 'ROS 2'")
        
        print(f"\n✓ Index verification completed for '{collection_name}'")
        print(f"  Total points: {collection_info.points_count}")
        print(f"  Vector size: {collection_info.config.params.vectors.size}")
        print(f"  Distance: {collection_info.config.params.vectors.distance}")
        
        return True
        
    except Exception as e:
        print(f"Error verifying Qdrant indexing: {str(e)}")
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Verify content indexing in Qdrant")
    parser.add_argument("--collection", default="book_content_en", help="Qdrant collection name")
    
    args = parser.parse_args()
    
    # Run async function
    success = asyncio.run(check_qdrant_indexing(args.collection))
    
    if success:
        print("\n✓ Content indexing verification PASSED")
    else:
        print("\n✗ Content indexing verification FAILED")
        exit(1)


if __name__ == "__main__":
    main()