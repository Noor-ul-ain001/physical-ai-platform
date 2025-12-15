---
id: 2
title: Create Implementation Plan for Physical AI Platform
stage: plan
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: physical-ai-platform
branch: 001-physical-ai-platform
user: noormudasir006
command: /sp.plan
labels: ["planning", "architecture", "technical-design", "full-stack", "rag-chatbot"]
links:
  spec: specs/001-physical-ai-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-platform/plan.md
  - specs/001-physical-ai-platform/research.md
  - specs/001-physical-ai-platform/data-model.md
  - specs/001-physical-ai-platform/contracts/openapi.yml
  - specs/001-physical-ai-platform/quickstart.md
tests:
  - Constitution Check: PASS (all Articles I-VI verified)
  - Research decisions validated (8 major technology choices)
  - Data model completeness check (6 entities with relationships)
---

## Prompt

User provided detailed technology stack for Physical AI educational platform implementation:

**Frontend**: Docusaurus 3.0, React 18, TypeScript 5.3, shadcn/ui, Framer Motion
**Button System**: 4 categories (Header, Chapter Action Bar, Inline, Sidebar) with consistent sizing/colors
**Backend**: FastAPI, Python 3.11, OpenAI Agents SDK, Pydantic v2
**Databases**: Neon Postgres (users, progress, bookmarks), Qdrant Cloud (vectors), Upstash Redis (cache)
**Auth**: Better Auth with extended user schema (hardware_profile, learning_goals)
**Translation**: Google Translate API with custom Urdu glossary
**Deployment**: Vercel (frontend), Railway (backend)

**User Stories** (from spec): Content access (P1), RAG chatbot (P2), Auth+Profile (P3), Personalization (P4), Translation (P5), AI Agents (P6)

## Response snapshot

Created comprehensive implementation plan with Phase 0 research and Phase 1 design artifacts:

**Plan Structure** (`plan.md`):
- Technical Context: TypeScript 5.3 + Python 3.11, Docusaurus 3.0 + FastAPI, full technology stack
- Constitution Check: ✅ PASS on all 6 articles (Embodied Intelligence, Spec-Driven, AI-Augmented, Personalization, RAG Requirements, Agent Skills)
- Project Structure: Frontend (Docusaurus), Backend (FastAPI), Agents (5 specialized), Deployment (GitHub Actions)
- Directory layout: frontend/src/components/buttons/, backend/src/api/, agents/DocusaurusContentAgent/, etc.

**Phase 0 Research** (`research.md`):
- 8 major technical decisions resolved:
  1. Docusaurus 3.0 + React 18 + TypeScript integration (SSG + MDX v3)
  2. shadcn/ui + Framer Motion button system (accessible, animated)
  3. OpenAI Agents SDK over LangChain (simpler, native function calling)
  4. Qdrant over Pinecone/Weaviate (free tier, 1GB storage)
  5. Better Auth over NextAuth (TypeScript-first, extended schema)
  6. Google Translate over DeepL (stronger Urdu support)
  7. Client-side personalization with server hydration
  8. Vercel + Railway deployment (optimized for Docusaurus + FastAPI)
- Each decision includes: rationale, alternatives considered, best practices, references

**Phase 1 Data Model** (`data-model.md`):
- 6 core entities: User, UserProgress, Bookmark, Highlight, ChatSession, ChatMessage
- Relationships: User (1) ----< (M) Progress/Bookmarks/Highlights/ChatSessions
- Validation rules: Email regex, password requirements, hardware profile enums
- Vector embeddings (Qdrant): 1536 dimensions, metadata (chapter_id, difficulty, hardware_requirement)
- Redis cache: Session tokens, personalization rules, translations (with TTLs)
- State transitions: User lifecycle, Chat session lifecycle, Chapter progress
- Migrations: Alembic for schema versioning

**Phase 1 API Contracts** (`contracts/openapi.yml`):
- 11 endpoints defined across 4 tags (Chat, Auth, Users, Personalization, Translation, Progress)
- Chat endpoints: `/api/chat` (full book), `/api/chat/selective` (text-only)
- Auth endpoints: `/api/users/signup`, `/api/users/me`
- Personalization: `/api/content/personalize` (chapter-specific rules)
- Translation: `/api/translate` (Urdu with code preservation)
- Progress: `/api/bookmarks`, `/api/progress` (GET/POST)
- Schemas: ChatRequest/Response, Citation, UserResponse, HardwareProfile, PersonalizationRules

**Phase 1 Quickstart** (`quickstart.md`):
- Prerequisites: Node.js 18+, Python 3.11, API keys (OpenAI, Qdrant, Neon, Google Translate)
- Setup steps: Frontend (npm install, env config), Backend (venv, pip, alembic), Qdrant (index content)
- Testing: Frontend (Jest, Playwright), Backend (pytest, hallucination tests)
- Common issues: Module imports, Qdrant connection, translation API, performance
- Development workflow: Create chapter → Index → Test → Deploy
- Agent system: `/agents docusaurus-spec` for content generation

**Constitutional Compliance**:
- Article I (Embodied Intelligence): Content progression digital→physical, real hardware specs (Jetson Orin, RealSense)
- Article II (Spec-Driven): Plan follows spec.md, tasks via `/sp.tasks`, quality gates (quickstart validation)
- Article III (RAG Zero-Hallucination): Hallucination test suite planned, citations required, <2s latency target
- Article IV (Agent Skills): 5 agents (Docusaurus, RAG, Personalization, UI, DevOps) with versioned prompts
- Article V (Personalization): Better Auth + hardware profiling, user-triggered personalization, Urdu translation
- Article VI (Governance): Cross-artifact consistency, ADR for significant decisions

## Outcome

- ✅ Impact: Complete technical architecture for full-stack educational platform, ready for `/sp.tasks` task breakdown
- 🧪 Tests: Constitution Check PASS (6/6 articles), 8 research decisions validated, data model completeness verified
- 📁 Files: 5 planning artifacts created (plan.md, research.md, data-model.md, openapi.yml, quickstart.md)
- 🔁 Next prompts: `/sp.tasks` to generate implementation task breakdown, `/agents rag-integration` to implement chatbot infrastructure, `/sp.implement` to execute tasks
- 🧠 Reflection: Comprehensive planning phase balances constitutional requirements (zero hallucinations, personalization, spec-driven) with practical technology choices (Qdrant free tier, Vercel/Railway, OpenAI Agents SDK). Button system architecture provides consistent UI/UX across 4 categories. Data model supports all 6 user stories with proper relationships and validation. API contracts enable frontend-backend integration. Quickstart ensures reproducible developer onboarding. Ready for task generation and incremental MVP delivery.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all Phase 0-1 artifacts
- Graders run and results (PASS/FAIL):
  - Constitution Check: PASS (all 6 articles verified)
  - Research completeness: PASS (8 decisions with rationale + alternatives)
  - Data model validity: PASS (6 entities, proper relationships, validation rules)
  - API contract coverage: PASS (11 endpoints cover all 6 user stories)
  - Quickstart reproducibility: PENDING (will validate during implementation)
- Prompt variant (if applicable): N/A - Initial implementation planning
- Next experiment (smallest change to try):
  1. Generate tasks with `/sp.tasks` for incremental MVP (Module 1 content + basic chatbot)
  2. Invoke `/agents rag-integration` to scaffold FastAPI backend with hallucination tests
  3. Test RAG chatbot with 50 out-of-scope questions to validate zero-hallucination requirement
  4. Implement button system with Framer Motion animations and accessibility testing
