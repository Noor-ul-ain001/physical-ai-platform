import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api.chat import router as chat_router # Import the chat router

# Configure basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Create a FastAPI app instance
app = FastAPI(
    title="AI Book RAG Chatbot API",
    description="API for 'The Book of AI-Driven Development' chatbot.",
    version="1.0.0",
)

# --- Middleware ---

# For a hackathon project, allowing all origins is convenient for development.
# In a production environment, you should restrict this to the specific domains
# where your frontend is hosted.
origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# --- API Endpoints ---

# Include the chat router
app.include_router(chat_router, prefix="/api")

@app.get("/", tags=["Health Check"])
async def root():
    """
    Root endpoint for health checks.
    """
    logger.info("Health check endpoint accessed.")
    return {"status": "ok", "message": "Welcome to the RAG Chatbot API!"}

if __name__ == "__main__":
    import uvicorn
    try:
        logger.info("Starting FastAPI application...")
        uvicorn.run(app, host="0.0.0.0", port=8000)
    except Exception as e:
        logger.error(f"Failed to start FastAPI application: {e}")
