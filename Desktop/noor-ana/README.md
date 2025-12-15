# Physical AI & Humanoid Robotics Educational Platform

The Physical AI & Humanoid Robotics Educational Platform is a comprehensive learning environment focused on embodied intelligence, robotics, and AI applications in physical systems.

## Table of Contents
- [Features](#features)
- [Tech Stack](#tech-stack)
- [Getting Started](#getting-started)
- [Development](#development)
- [Deployment](#deployment)
- [Architecture](#architecture)

## Features

### 1. Curriculum Access
- 13-week curriculum across 4 modules
- Responsive Docusaurus-based interface
- Interactive code examples and components
- Hardware specification tables

### 2. RAG Chatbot
- Full-book and selective context modes
- Zero hallucination guarantee
- Source citations for all responses
- <2s response time target

### 3. Authentication & Profile
- User signup/login with Better Auth
- Hardware and experience profiling
- Learning goal tracking

### 4. Content Personalization
- Adaptive content based on user profile
- Hardware-specific examples
- Experience-level appropriate complexity

### 5. Translation
- Urdu translation capabilities
- Code preservation during translation
- Technical term consistency

### 6. AI Agents
- Content generation agent
- RAG optimization agent
- Personalization rules agent
- UI component generation agent

## Tech Stack

### Frontend
- **Framework**: Docusaurus 3.0 with React 18 & TypeScript
- **UI Components**: shadcn/ui with Tailwind CSS
- **Animations**: Framer Motion
- **Documentation**: MDX format with interactive components

### Backend
- **Framework**: FastAPI 0.104+ with Python 3.11
- **Database**: Neon Postgres (serverless)
- **Vector DB**: Qdrant Cloud
- **Cache**: Redis (Upstash)
- **AI**: OpenAI API, Google Translate API

### Deployment
- **Frontend**: Vercel
- **Backend**: Railway
- **CI/CD**: GitHub Actions

## Getting Started

### Prerequisites
- Node.js 18+
- Python 3.11+
- Git

### Frontend Setup
```bash
cd frontend
npm install
cp .env.local.example .env.local
npm run start
```

### Backend Setup
```bash
cd backend
python3.11 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
cp .env.example .env
# Update .env with your credentials
uvicorn src.main:app --reload
```

### Environment Variables
- `NEXT_PUBLIC_API_URL`: Frontend API endpoint
- `DATABASE_URL`: Neon Postgres connection string
- `QDRANT_URL` / `QDRANT_API_KEY`: Qdrant Cloud credentials
- `OPENAI_API_KEY`: OpenAI API key
- `REDIS_URL`: Upstash Redis URL
- `GOOGLE_TRANSLATE_API_KEY`: Google Translate API key
- `BETTER_AUTH_SECRET`: Better Auth secret

## Development

### Adding Content
New curriculum content should be added as MDX files in `frontend/docs/` following the existing structure. Each module directory contains week-specific content files.

### Creating Components
Reusable components go in `frontend/src/components/` with appropriate subdirectories. Common components are in the `common` directory, while specialized components like those for chat, auth, or personalization have dedicated directories.

### Backend Services
New backend services should follow the structure in `backend/src/services/`. Each service should be self-contained with clear interfaces and proper error handling.

### API Endpoints
API endpoints are organized by functionality in `backend/src/api/`. New endpoints should follow the existing patterns with proper request/response validation using Pydantic models.

## Deployment

### Frontend (Vercel)
The frontend is configured for deployment on Vercel. The `vercel.json` configuration handles routing and security headers.

### Backend (Railway)
The backend is configured for deployment on Railway. The Dockerfile defines the container image, and environment variables are set through the Railway dashboard.

### CI/CD
GitHub Actions workflows in `.github/workflows/` handle automated testing and deployment. Workflows trigger on pushes to main and pull requests to ensure code quality.

## Architecture

### Frontend Architecture
- Docusaurus serves static content with React components
- Context providers manage global state (translations, user profile)
- Custom Docusaurus theme wraps curriculum content with platform features
- Interactive components enhance learning experience

### Backend Architecture
- FastAPI handles HTTP requests with async support
- Service layer contains business logic
- Repository pattern for database interactions
- Qdrant handles vector storage for RAG
- Redis provides caching and session management

### Data Flow
1. User requests curriculum content
2. Docusaurus serves static files
3. Interactive components fetch data via API
4. Backend processes requests using services
5. Services interact with databases and external APIs
6. Responses are cached where appropriate

## Contributing

We welcome contributions to the Physical AI & Humanoid Robotics Educational Platform. Please read our contributing guidelines for details on our code of conduct and the process for submitting pull requests.

## Support

If you encounter issues, please check our documentation first. For additional support, open an issue in our GitHub repository with detailed information about the problem and steps to reproduce.