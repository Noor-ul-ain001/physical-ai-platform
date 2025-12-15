# Quickstart Guide: Physical AI & Humanoid Robotics Platform

**Feature**: 001-physical-ai-platform
**Last Updated**: 2025-12-15

## Prerequisites

- **Node.js**: v18 or higher
- **Python**: 3.11 or higher
- **Git**: For version control
- **PostgreSQL**: Via Neon (cloud) or local installation
- **Redis**: Via Upstash (cloud) or local installation
- **API Keys**:
  - OpenAI API key (for embeddings + chat)
  - Qdrant Cloud API key + URL
  - Neon Postgres connection string
  - Google Cloud Translation API key (for Urdu)
  - Better Auth secret

## Environment Setup

### 1. Clone Repository

```bash
git checkout 001-physical-ai-platform
```

### 2. Frontend Setup (Docusaurus)

```bash
cd frontend

# Install dependencies
npm install

# Copy environment template
cp .env.local.example .env.local

# Edit .env.local with your API endpoints
# NEXT_PUBLIC_API_URL=http://localhost:8000
# NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3000

# Start development server
npm run start
```

**Verify**: Open http://localhost:3000, should see homepage with curriculum modules.

### 3. Backend Setup (FastAPI)

```bash
cd backend

# Create virtual environment
python3.11 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.example .env

# Edit .env with your credentials:
# DATABASE_URL=postgresql://user:pass@neon-host/db
# QDRANT_URL=https://your-cluster.qdrant.io
# QDRANT_API_KEY=your-key
# OPENAI_API_KEY=sk-...
# REDIS_URL=redis://default:pass@upstash-url
# GOOGLE_TRANSLATE_API_KEY=your-key
# BETTER_AUTH_SECRET=generate-random-secret

# Run database migrations
alembic upgrade head

# Start development server
uvicorn src.main:app --reload --port 8000
```

**Verify**: Open http://localhost:8000/docs, should see FastAPI Swagger UI.

### 4. Initialize Vector Database (Qdrant)

```bash
cd backend

# Run embedding script (index sample content)
python scripts/index_content.py --source ../frontend/docs/ --collection book_content_en

# Verify vectors indexed
python scripts/check_qdrant.py
```

**Expected Output**: "Indexed 104 chunks from 16 chapters"

### 5. Test RAG Chatbot

```bash
# Terminal 1: Backend running
uvicorn src.main:app --reload

# Terminal 2: Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "topK": 5}'
```

**Expected Response**:
```json
{
  "answer": "ROS 2 is a middleware framework...",
  "citations": [
    {"source": "module-1/week-1-introduction", "excerpt": "...", "similarity": 0.92}
  ],
  "confidence": 0.95,
  "retrievalTimeMs": 1250
}
```

### 6. Test Authentication

```bash
# Create test user
curl -X POST http://localhost:8000/api/users/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "Test1234"}'

# Get auth token
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "Test1234"}'

# Use token for authenticated request
curl -X GET http://localhost:8000/api/users/me \
  -H "Authorization: Bearer YOUR_TOKEN"
```

## Running Tests

### Frontend Tests

```bash
cd frontend

# Unit tests (Jest)
npm run test

# E2E tests (Playwright)
npm run test:e2e

# Component tests with coverage
npm run test:coverage
```

### Backend Tests

```bash
cd backend

# Unit tests
pytest tests/unit/ -v

# Integration tests (requires running services)
pytest tests/integration/ -v

# Critical hallucination tests
pytest tests/integration/test_hallucination.py -v

# All tests with coverage
pytest --cov=src --cov-report=html
```

## Common Issues

**Issue**: `ModuleNotFoundError: No module named 'openai_agents_sdk'`
**Fix**: Install latest SDK: `pip install openai-agents-sdk --upgrade`

**Issue**: Qdrant connection timeout
**Fix**: Check QDRANT_URL and QDRANT_API_KEY in .env, verify cluster is active in Qdrant Cloud dashboard

**Issue**: Frontend buttons not showing
**Fix**: Clear browser cache, ensure `npm run build` completed successfully

**Issue**: Translation not working
**Fix**: Verify GOOGLE_TRANSLATE_API_KEY is valid, check API quotas in Google Cloud Console

**Issue**: Performance <3s page load target
**Fix**: Run `npm run build && npm run serve` for production build (dev mode is slower)

## Development Workflow

1. **Create new chapter**: Add MDX file in `frontend/docs/module-X/`
2. **Add interactive component**: Create in `frontend/src/components/`, import in MDX
3. **Index new content**: Run `python scripts/index_content.py` after adding chapters
4. **Test personalization**: Update user profile in frontend, click "Personalize This Chapter"
5. **Test translation**: Click "Translate to Urdu", verify code blocks unchanged

## Agent System (AI-Driven Development)

To use AI agents for content generation:

```bash
# List available agents
/agents

# Generate new chapter
/agents docusaurus-spec "Create Chapter: ROS 2 Services with code examples"

# Implement RAG feature
/agents rag-integration "Add selective context retrieval"
```

## Next Steps

1. **Content Creation**: Use `/agents docusaurus-spec` to generate Module 1 chapters
2. **Testing**: Run full test suite (`npm run test && pytest`)
3. **Deployment**: Follow deployment guide in `docs/deployment.md`
4. **Monitoring**: Set up Sentry for error tracking, Vercel Analytics for performance

## Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com
- **Qdrant Docs**: https://qdrant.tech/documentation
- **Better Auth Docs**: https://better-auth.com/docs
- **OpenAI Agents SDK**: https://platform.openai.com/docs/assistants
