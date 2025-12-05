# Research: Backend Deployment Platform

**Feature**: AI-Driven Development Book & RAG Chatbot

## Decision

We will use **Vercel** for deploying the FastAPI backend.

## Rationale

For a hackathon-style project, speed of iteration and ease of deployment are critical. Vercel provides a seamless developer experience with the following benefits:

- **Git Integration:** Connects directly to the GitHub repository for automatic deployments on every push to the main branch.
- **Serverless Functions:** Natively supports Python (WSGI/ASGI), allowing the FastAPI application to be deployed as a serverless function without managing infrastructure.
- **Generous Free Tier:** The free "Hobby" plan is sufficient for this project's needs, including a fair amount of serverless function invocations.
- **Simplicity:** Configuration is minimal compared to more complex cloud providers like AWS or GCP, making it ideal for rapid setup.

## Alternatives Considered

- **Heroku:** A strong contender, but recent changes to its free tier make it less attractive for small projects and prototypes.
- **AWS Lambda + API Gateway:** Extremely powerful and scalable, but the learning curve and setup complexity are significantly higher and not ideal for the rapid pace of a hackathon.
- **Simple Virtual Machine (e.g., DigitalOcean Droplet, EC2):** Provides full control but requires manual setup of the OS, web server (like Gunicorn/Uvicorn), process management, and CI/CD pipelines, which is overkill for this project.
