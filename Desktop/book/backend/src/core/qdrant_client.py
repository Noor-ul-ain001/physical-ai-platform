import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Get Qdrant credentials from environment variables
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

# Initialize the Qdrant client
# This will raise an error if the URL is not set, which is intended.
qdrant_client = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
)

def get_qdrant_client():
    """
    Returns a configured instance of the Qdrant client.
    """
    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable not set.")
    
    return qdrant_client

# You can add other utility functions related to Qdrant here
# For example, a function to create a collection if it doesn't exist.

if __name__ == '__main__':
    # A simple test to verify the connection
    try:
        collections = qdrant_client.get_collections()
        print("Successfully connected to Qdrant.")
        print("Available collections:", collections)
    except Exception as e:
        print(f"Failed to connect to Qdrant. Please check your QDRANT_URL and QDRANT_API_KEY in the .env file.")
        print(f"Error: {e}")

