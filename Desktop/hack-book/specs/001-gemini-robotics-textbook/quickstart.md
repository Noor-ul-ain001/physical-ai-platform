# Quickstart for Gemini Robotics Textbook

**Input**: `plan.md` for feature `001-gemini-robotics-textbook`

This document provides a brief overview of how to get started with the project.

## Prerequisites

*   Node.js 18+
*   Python 3.11+
*   Git
*   Access to Google Gemini API

## Getting Started

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd physical-ai-textbook
    ```

2.  **Install dependencies:**
    *   **Frontend:**
        ```bash
        cd frontend
        npm install
        ```
    *   **Backend:**
        ```bash
        cd backend
        pip install -r requirements.txt
        ```

3.  **Configure environment variables:**
    *   Create a `.env` file in the `backend` directory.
    *   Add your Google Gemini API key and other necessary credentials.
        ```
        GEMINI_API_KEY=your_api_key
        DATABASE_URL=your_database_url
        QDRANT_URL=your_qdrant_url
        QDRANT_API_KEY=your_qdrant_api_key
        ```

4.  **Run the application:**
    *   **Start the backend server:**
        ```bash
        cd backend
        uvicorn main:app --reload
        ```
    *   **Start the frontend development server:**
        ```bash
        cd frontend
        npm start
        ```

5.  **Access the application:**
    *   Open your browser and navigate to `http://localhost:3000`.

## Project Structure

Refer to the `Project Structure` section in the `spec.md` file for a detailed overview of the repository layout.
