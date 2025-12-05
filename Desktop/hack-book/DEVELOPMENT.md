# Development Setup

This document describes how to set up the development environment for the Gemini-Powered Physical AI Textbook.

## Prerequisites

- Node.js (version 18 or higher)
- npm
- Python (version 3.11 or higher)
- pip
- Git

## Frontend Setup

1.  Navigate to the `frontend` directory:
    ```bash
    cd frontend
    ```

2.  Install the dependencies:
    ```bash
    npm install
    ```

3.  Start the development server:
    ```bash
    npm start
    ```

The frontend will be available at `http://localhost:3000`.

## Backend Setup

1.  Navigate to the `backend` directory:
    ```bash
    cd backend
    ```

2.  It is recommended to create a virtual environment:
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```

3.  Install the Python dependencies:
    ```bash
    pip install -r requirements.txt
    ```

4.  Create a `.env` file and add your environment variables (e.g., `GEMINI_API_KEY`).

5.  Start the backend server:
    ```bash
    uvicorn main:app --reload
    ```

The backend API will be available at `http://localhost:8000`.
