# Implementation Plan: Physical AI & Humanoid Robotics Educational Platform

**Branch**: `001-physical-ai-platform` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-platform/spec.md`

## Summary

Build a comprehensive educational platform for Physical AI & Humanoid Robotics using Docusaurus v3.0 with React 18/TypeScript, featuring an integrated RAG chatbot (FastAPI + Qdrant + Neon Postgres), Better Auth authentication with user profiling, chapter-level personalization based on hardware/skill background, Urdu translation capabilities, and a sophisticated button-based UI system for interactive learning. Platform serves a 13-week curriculum across 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) with executable code snippets, hardware requirement tables, and AI agent-driven content generation.

## Technical Context

**Language/Version**: TypeScript 5.3 (Frontend), Python 3.11 (Backend)

**Primary Dependencies**:
- **Frontend**: Docusaurus 3.0, React 18, shadcn/ui, Framer Motion, Better Auth (client)
- **Backend**: FastAPI 0.104+, Pydantic v2, OpenAI Agents SDK, asyncpg, aiohttp
- **Databases**: Neon Serverless Postgres, Qdrant Cloud (Free Tier), Redis (Upstash)
- **Translation**: Google Translate API with custom technical glossary

**Storage**:
- **Neon Postgres**: Users, profiles, progress, bookmarks, highlights, chat sessions
- **Qdrant Cloud**: Vector embeddings (English + Urdu), 500-token chunks, semantic metadata
- **Redis (Upstash)**: Session caching, personalized content cache, rate limiting

**Testing**:
- **Frontend**: Jest + React Testing Library for components, Playwright for E2E
- **Backend**: pytest + pytest-asyncio for API, hallucination detection suite
- **Integration**: Cypress for full user flows (chat, personalization, translation)

**Target Platform**:
- **Frontend Deployment**: Vercel (SSG with edge functions for API proxying)
- **Backend Deployment**: Railway (FastAPI with auto-scaling)
- **CDN**: Vercel Edge Network for static assets
- **Mobile**: Responsive web (320px-1920px), no native apps

**Project Type**: Full-stack web application (Docusaurus frontend + FastAPI backend microservice)

**Performance Goals**:
- Page load: <3 seconds (p95)
- Chatbot response: <2 seconds (p95)
- Concurrent users: 1000 without degradation
- Mobile responsiveness: 320px minimum width
- SEO score: >90 (Lighthouse)

**Constraints**:
- Qdrant free tier: 1GB storage, 100k vectors (optimize chunking)
- Neon free tier: 0.5GB storage (efficient schema)
- OpenAI API costs: Budget for embeddings + chat completions
- Translation API: Rate limits for Urdu translation
- Zero hallucinations (constitutional critical requirement)

**Scale/Scope**:
- 13-week curriculum (~52 chapters across 4 modules)
- MVP: Module 1 (Weeks 1-4, ~16 chapters)
- Expected users: 100-500 learners in first semester
- Content update frequency: Quarterly for hardware changes, weekly for corrections

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Article I: Embodied Intelligence First

- ✅ **Content Progression**: Curriculum structured to progress from digital AI foundations to physical robotics embodiment
- ✅ **Real-World Constraints**: Code snippets demonstrate latency considerations, hardware specifications include real equipment (Jetson Orin, RealSense)
- ✅ **Physical Grounding**: Interactive features (simulation runners, hardware toggles) ground concepts in physical scenarios
- **Compliance**: PASS

### Article II: AI/Spec-Driven Book Creation

- ✅ **Specification First**: This plan follows `/sp.specify` output (spec.md with 6 user stories, 47 requirements)
- ✅ **Architectural Documentation**: This plan documents technical architecture before implementation
- ✅ **Task Derivation**: Tasks will be generated via `/sp.tasks` with file paths and test cases
- ✅ **Quality Gates**: Quickstart validation, cross-artifact consistency checks planned
- **Compliance**: PASS

### Article III: The Integrated RAG Chatbot System

- ✅ **Accessibility**: Chatbot accessible within 2 clicks (ChatbotToggle button in Chapter Action Bar)
- ✅ **Citations**: Response model includes `citations: List[Citation]` with source references
- ✅ **Performance**: <2s latency target with async FastAPI + Qdrant optimization
- ✅ **Zero Hallucinations**: Hallucination detection test suite (6 tests) planned, zero-tolerance enforced
- ✅ **Selective Context**: `/api/chat/selective` endpoint for text-selection-only retrieval
- ✅ **Technology Stack**: FastAPI + OpenAI Agents SDK + Qdrant + Neon Postgres as specified
- **Compliance**: PASS

### Article IV: Reusable Intelligence & Agent Skills

- ✅ **Subagent Creation**: 5 specialized agents planned (DocusaurusContentAgent, RAGIntegrationAgent, PersonalizationAgent, UIComponentAgent, DevOpsAgent)
- ✅ **Versioned Prompts**: Agent directory structure includes `system_prompt.md`, `skills.json`, `examples/`
- ✅ **Skill Documentation**: Each agent has `button_specs.md` for UI integration, skills with input/output contracts
- ✅ **Knowledge Preservation**: PHR generation mandated for every agent invocation
- **Compliance**: PASS

### Article V: User Authentication & Personalization

- ✅ **Authentication Framework**: Better Auth with email/password + OAuth providers
- ✅ **Background Profiling**: Signup questionnaire captures hardware access, programming level, robotics experience
- ✅ **Personalized Gateways**: "Personalize This Chapter" button triggers content adaptation
- ✅ **Translation**: "Translate to Urdu" button with code integrity preservation
- ✅ **User Control**: Personalization is opt-in, toggle to revert to canonical view
- **Compliance**: PASS

### Article VI: Governance & Evolution

- ✅ **Constitutional Authority**: This plan verified against Articles I-VI
- ✅ **Quality Enforcement**: `/sp.analyze` cross-artifact checks planned
- ✅ **Amendment Process**: Architectural decisions will be documented as ADRs when significant
- **Compliance**: PASS

**OVERALL GATE STATUS**: ✅ PASS - All constitutional requirements satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-platform/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (research findings)
├── data-model.md        # Phase 1 output (database schema)
├── quickstart.md        # Phase 1 output (developer setup guide)
├── contracts/           # Phase 1 output (API specs)
│   └── openapi.yml      # FastAPI endpoints OpenAPI schema
├── checklists/          # Quality validation
│   └── requirements.md  # Spec validation checklist
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus + React)
frontend/
├── docusaurus.config.ts          # Docusaurus configuration
├── sidebars.ts                    # Sidebar navigation structure
├── src/
│   ├── components/                # React components
│   │   ├── buttons/               # Button system
│   │   │   ├── PersonalizationButton.tsx
│   │   │   ├── TranslationButton.tsx
│   │   │   ├── ChatbotToggle.tsx
│   │   │   ├── BookmarkButton.tsx
│   │   │   ├── TextSelectionToolbar.tsx
│   │   │   ├── CodeBlockActions.tsx
│   │   │   └── HardwareToggleButtons.tsx
│   │   ├── chat/                  # Chatbot components
│   │   │   ├── ChatbotWidget.tsx
│   │   │   ├── ContextModeToggle.tsx
│   │   │   └── ChatMessage.tsx
│   │   ├── auth/                  # Authentication
│   │   │   ├── UserAuthButton.tsx
│   │   │   ├── SignupForm.tsx
│   │   │   ├── SigninForm.tsx
│   │   │   └── ProfileQuestionnaire.tsx
│   │   ├── personalization/       # Personalization
│   │   │   ├── UserProfilePanel.tsx
│   │   │   ├── PersonalizationModal.tsx
│   │   │   └── ContentAdapter.tsx
│   │   ├── common/                # Shared UI
│   │   │   ├── Button.tsx         # Base button component
│   │   │   ├── Modal.tsx
│   │   │   ├── ProgressTracker.tsx
│   │   │   └── NotificationBell.tsx
│   │   └── mdx/                   # MDX enhancements
│   │       ├── InteractiveCodeBlock.tsx
│   │       ├── HardwareTable.tsx
│   │       └── DiagramViewer.tsx
│   ├── css/                       # Styles
│   │   ├── custom.css             # Custom theme
│   │   ├── buttons.module.css     # Button styling
│   │   └── robotics-theme.css     # Circuit board patterns
│   ├── pages/                     # Custom pages
│   │   ├── index.tsx              # Homepage
│   │   ├── dashboard.tsx          # User dashboard
│   │   ├── signup.tsx             # Signup page
│   │   └── signin.tsx             # Signin page
│   └── theme/                     # Docusaurus theme overrides
│       ├── Navbar/                # Custom navbar
│       ├── Footer/                # Custom footer
│       └── DocItem/               # Chapter page enhancements
├── docs/                          # Educational content (MDX)
│   ├── intro.md                   # Course introduction
│   ├── module-1/                  # ROS 2 Fundamentals (Weeks 1-4)
│   │   ├── week-1-introduction.md
│   │   ├── week-2-nodes-topics.md
│   │   ├── week-3-services-actions.md
│   │   └── week-4-tf-urdf.md
│   ├── module-2/                  # Simulation (Weeks 5-7)
│   │   ├── week-5-gazebo-basics.md
│   │   ├── week-6-unity-ml-agents.md
│   │   └── week-7-sensor-simulation.md
│   ├── module-3/                  # NVIDIA Isaac Sim (Weeks 8-10)
│   │   ├── week-8-isaac-intro.md
│   │   ├── week-9-synthetic-data.md
│   │   └── week-10-robot-learning.md
│   └── module-4/                  # VLA Models (Weeks 11-13)
│       ├── week-11-vision-language.md
│       ├── week-12-action-models.md
│       └── week-13-embodied-ai.md
├── static/                        # Static assets
│   ├── img/                       # Images, diagrams
│   └── hardware/                  # Hardware spec sheets
├── package.json                   # Dependencies
├── tsconfig.json                  # TypeScript config
└── .env.local.example             # Environment template

# Backend (FastAPI)
backend/
├── src/
│   ├── api/                       # API routes
│   │   ├── chat.py                # RAG chatbot endpoints
│   │   ├── users.py               # User management
│   │   ├── content.py             # Personalization, translation
│   │   ├── bookmarks.py           # Bookmark management
│   │   ├── progress.py            # Progress tracking
│   │   └── auth.py                # Authentication endpoints
│   ├── core/                      # Core utilities
│   │   ├── config.py              # Configuration
│   │   ├── database.py            # Postgres connection
│   │   ├── qdrant_client.py       # Qdrant setup
│   │   ├── redis_client.py        # Redis connection
│   │   └── embeddings.py          # Vector embedding generation
│   ├── services/                  # Business logic
│   │   ├── rag.py                 # RAG retrieval + generation
│   │   ├── personalization.py     # Content adaptation
│   │   ├── translation.py         # Urdu translation
│   │   ├── progress_tracker.py    # User progress
│   │   └── notifications.py       # User alerts
│   ├── models/                    # Pydantic models
│   │   ├── user.py                # User, UserProfile
│   │   ├── chat.py                # ChatRequest, ChatResponse, Citation
│   │   ├── content.py             # Chapter, PersonalizationRequest
│   │   └── common.py              # Shared models
│   ├── db/                        # Database layer
│   │   ├── models.py              # SQLAlchemy models
│   │   ├── repositories/          # Data access
│   │   │   ├── user_repository.py
│   │   │   ├── chat_repository.py
│   │   │   └── progress_repository.py
│   │   └── migrations/            # Alembic migrations
│   ├── agents/                    # OpenAI Agents SDK integration
│   │   ├── chat_agent.py          # RAG orchestration agent
│   │   └── tools.py               # Custom tools (book navigation)
│   └── main.py                    # FastAPI app entry
├── tests/
│   ├── integration/               # Integration tests
│   │   ├── test_chat.py
│   │   ├── test_hallucination.py  # Critical zero-tolerance tests
│   │   └── test_personalization.py
│   └── unit/                      # Unit tests
│       ├── test_rag_service.py
│       └── test_translation.py
├── requirements.txt               # Python dependencies
├── pyproject.toml                 # Poetry config
├── alembic.ini                    # Database migrations
└── .env.example                   # Environment template

# Agent System
agents/
├── DocusaurusContentAgent/
│   ├── system_prompt.md           # Agent instructions
│   ├── skills.json                # Capability definitions
│   ├── examples/                  # Example inputs/outputs
│   └── button_specs.md            # Button integration guide
├── RAGIntegrationAgent/
│   ├── system_prompt.md
│   ├── skills.json
│   └── hallucination_tests.md     # Test scenarios
├── PersonalizationAgent/
│   ├── system_prompt.md
│   ├── skills.json
│   └── content_rules.md           # Personalization logic
├── UIComponentAgent/
│   ├── system_prompt.md
│   ├── skills.json
│   └── design_system.md           # UI/UX guidelines
└── DevOpsAgent/
    ├── system_prompt.md
    ├── skills.json
    └── deployment_config.md       # CI/CD setup

# Deployment
.github/
└── workflows/
    ├── frontend-deploy.yml        # Vercel deployment
    ├── backend-deploy.yml         # Railway deployment
    └── tests.yml                  # CI tests (hallucination, E2E)

# Configuration
.env.example                       # Environment variables template
docker-compose.yml                 # Local development setup
```

**Structure Decision**: Full-stack web application with clear frontend/backend separation. Frontend is Docusaurus static site with React components for interactivity. Backend is FastAPI microservice handling dynamic features (chat, personalization, translation). Agent system is separate directory for AI-driven content generation and maintenance.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | Constitution Check PASSED | No violations to justify |

---

**Phase 0 follows in research.md**
**Phase 1 artifacts: data-model.md, contracts/openapi.yml, quickstart.md**
**Phase 2 (tasks.md) generated by /sp.tasks command**
