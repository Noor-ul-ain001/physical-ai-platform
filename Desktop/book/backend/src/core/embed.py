import os
import uuid
import argparse
from openai import OpenAI
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

# --- CONFIGURATION ---
COLLECTION_NAME = "ai-book"
EMBEDDING_MODEL = "text-embedding-3-small"

def get_clients():
    """Initializes and returns OpenAI and Qdrant clients."""
    load_dotenv()
    
    openai_api_key = os.getenv("OPENAI_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not all([openai_api_key, qdrant_url, qdrant_api_key]):
        raise ValueError("API keys and URLs for OpenAI and Qdrant must be set in the .env file.")

    openai_client = OpenAI(api_key=openai_api_key)
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    
    return openai_client, qdrant_client

def read_file(filepath):
    """Reads content from a file."""
    print(f"Reading content from {filepath}...")
    with open(filepath, 'r', encoding='utf-8') as f:
        return f.read()

def chunk_text(text, chunk_by="paragraph"):
    """Chunks text by paragraph."""
    # A more sophisticated chunking strategy would be used in production
    print(f"Chunking text by {chunk_by}...")
    if chunk_by == "paragraph":
        return [p.strip() for p in text.split('\n\n') if p.strip()]
    else:
        raise NotImplementedError(f"Chunking by '{chunk_by}' is not supported.")

def get_embeddings(texts, client, model=EMBEDDING_MODEL):
    """Generates embeddings for a list of texts using OpenAI."""
    if not texts:
        return []
    print(f"Generating embeddings for {len(texts)} chunks using {model}...")
    response = client.embeddings.create(input=texts, model=model)
    return [item.embedding for item in response.data]

def upsert_to_qdrant(client, collection_name, texts, vectors, source, recreate_collection=False):
    """Upserts text chunks and their vectors to a Qdrant collection."""
    if not texts or not vectors:
        print("No texts or vectors to upsert. Skipping.")
        return

    print(f"Upserting {len(texts)} vectors into Qdrant collection '{collection_name}' (Source: {source})...")
    
    # Check if collection exists and recreate if flag is set
    collection_exists = client.collection_exists(collection_name=collection_name)
    if recreate_collection and collection_exists:
        print(f"Recreating collection '{collection_name}'...")
        client.delete_collection(collection_name=collection_name)
        collection_exists = False # Mark as not existing after deletion

    if not collection_exists:
        print(f"Collection '{collection_name}' not found. Creating it...")
        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=len(vectors[0]), distance=models.Distance.COSINE),
        )

    # Prepare points for upsertion
    points = []
    for i, text in enumerate(texts):
        points.append(
            models.PointStruct(
                id=str(uuid.uuid4()),
                vector=vectors[i],
                payload={"text": text, "source": source}
            )
        )
    
    # Upsert points
    client.upsert(
        collection_name=collection_name,
        points=points,
        wait=True
    )
    print("Upsert complete.")

def main():
    parser = argparse.ArgumentParser(description="Embeds Docusaurus content into Qdrant.")
    parser.add_argument("--docs-path", type=str, default="..\\..\\frontend\\docs",
                        help="Path to the Docusaurus docs directory (e.g., 'frontend/docs').")
    parser.add_argument("--recreate-collection", action="store_true",
                        help="If set, the Qdrant collection will be deleted and recreated.")
    args = parser.parse_args()

    try:
        openai_client, qdrant_client = get_clients()
        
        docs_dir = os.path.abspath(args.docs_path)
        print(f"Processing documents from: {docs_dir}")

        processed_files_count = 0
        for root, _, files in os.walk(docs_dir):
            for file in files:
                if file.endswith(('.md', '.mdx')):
                    filepath = os.path.join(root, file)
                    # For Qdrant source, use path relative to docs_dir
                    relative_path = os.path.relpath(filepath, docs_dir)

                    content = read_file(filepath)
                    chunks = chunk_text(content)
                    embeddings = get_embeddings(chunks, openai_client)
                    
                    if embeddings: # Only upsert if embeddings were successfully generated
                        upsert_to_qdrant(qdrant_client, COLLECTION_NAME, chunks, embeddings, source=relative_path, 
                                         recreate_collection=(processed_files_count == 0 and args.recreate_collection))
                        processed_files_count += 1
                    else:
                        print(f"Skipping {filepath}: No embeddings generated.")

        if processed_files_count > 0:
            print(f"\nEmbedding pipeline finished successfully! Processed {processed_files_count} files.")
        else:
            print("\nNo markdown/MDX files found or processed.")

    except Exception as e:
        print(f"\nAn error occurred: {e}")
        print("Please ensure your .env file is correctly set up with OpenAI and Qdrant credentials and paths are correct.")

if __name__ == '__main__':
    main()
