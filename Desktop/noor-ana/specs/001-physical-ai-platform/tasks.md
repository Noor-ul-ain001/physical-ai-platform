# Tasks: Physical AI & Humanoid Robotics Educational Platform

**Feature**: 001-physical-ai-platform
**Branch**: `001-physical-ai-platform`
**Last Updated**: 2025-12-15

## Overview

This task breakdown implements the Physical AI & Humanoid Robotics educational platform organized by user story priority. Each phase groups related tasks for efficient execution with clear dependencies and parallel execution opportunities.

**Development Approach**:
- **Phase 1 (Setup)**: Project initialization - blocking prerequisites
- **Phase 2 (Foundational)**: Database, authentication, core infrastructure
- **Phase 3-8**: User Stories P1-P6 implementation (independently testable)
- **Phase 9 (Polish)**: Cross-cutting concerns, optimization

**Task Format**: `- [ ] [TaskID] [P?] [Story?] Description (file: path)`
- **[P]** = Parallelizable with other [P] tasks in same phase
- **[US1]**, **[US2]**, etc. = User story association
- File paths reference the project structure from plan.md

## User Story Mapping

| Phase | User Story | Priority | Tasks | Independent Test |
|-------|------------|----------|-------|------------------|
| 3 | Content Access | P1 | T021-T035 | Navigate to homepage, browse modules, read chapter <3s |
| 4 | RAG Chatbot | P2 | T036-T047 | Ask 10 questions, verify >90% accuracy, <2s response |
| 5 | Auth & Profile | P3 | T048-T053 | Signup, questionnaire, verify profile stored |
| 6 | Personalization | P4 | T054-T061 | Personalize chapter, verify adaptation by profile |
| 7 | Translation | P5 | T062-T067 | Translate to Urdu, verify code blocks unchanged |
| 8 | AI Agents | P6 | T068-T077 | Invoke agent, verify valid MDX output |

---

## Phase 1: Project Initialization & Setup

**Goal**: Create repository structure, configure tooling, establish development environment

### Repository & Configuration

- [X] [T001] [P] [Setup] Initialize Git repository with .gitignore for Node/Python (file: `.gitignore`)
- [X] [T002] [P] [Setup] Create frontend/ directory with Docusaurus 3.0 scaffolding (file: `frontend/package.json`)
- [X] [T003] [P] [Setup] Create backend/ directory with FastAPI structure (file: `backend/src/main.py`)
- [X] [T004] [P] [Setup] Set up environment variable templates (files: `frontend/.env.local.example`, `backend/.env.example`)

**Dependency**: All Phase 2+ tasks depend on T001-T004 completing

---

## Phase 2: Foundational Infrastructure

**Goal**: Establish database schema, authentication, core services required by all user stories

### Database Setup

- [ ] [T005] [Found] Create Neon Postgres database and obtain connection string (external: Neon Cloud)
- [ ] [T006] [Found] Create Qdrant Cloud collection `book_content_en` with 1536 dimensions (external: Qdrant Cloud)
- [ ] [T007] [Found] Create Upstash Redis instance and obtain URL (external: Upstash)
- [X] [T008] [Found] Configure Alembic for database migrations (file: `backend/alembic.ini`)
- [X] [T009] [Found] Define SQLAlchemy models: User, UserProgress, Bookmark, Highlight (file: `backend/src/db/models.py`)
- [X] [T010] [Found] Create initial migration with User table (file: `backend/src/db/migrations/versions/001_initial.py`)
- [ ] [T011] [Found] Run migration: `alembic upgrade head` (command)

**Test**: Connect to databases, verify tables created

### Core Backend Services

- [ ] [T012] [P] [Found] Configure FastAPI app with CORS middleware (file: `backend/src/main.py`)
- [ ] [T013] [P] [Found] Create database connection pool manager (file: `backend/src/core/database.py`)
- [ ] [T014] [P] [Found] Create Qdrant client with async support (file: `backend/src/core/qdrant_client.py`)
- [ ] [T015] [P] [Found] Create Redis client with connection pooling (file: `backend/src/core/redis_client.py`)
- [ ] [T016] [P] [Found] Implement embeddings service with text-embedding-3-small (file: `backend/src/core/embeddings.py`)

**Test**: `pytest backend/tests/unit/test_connections.py` - verify all DB connections succeed

### Authentication (Better Auth)

- [ ] [T017] [Found] Install Better Auth library and configure with Neon Postgres (file: `backend/src/auth/config.py`)
- [ ] [T018] [Found] Create `/api/users/signup` endpoint with password hashing (file: `backend/src/api/auth.py`)
- [ ] [T019] [Found] Create `/api/users/login` endpoint with session management (file: `backend/src/api/auth.py`)
- [ ] [T020] [Found] Create `/api/users/me` endpoint with bearer token validation (file: `backend/src/api/auth.py`)

**Test**: Signup user, login, access `/api/users/me` with token

**Dependency**: All user story phases (3-8) depend on Phase 2 completing

---

## Phase 3: User Story 1 - Content Access (P1)

**Goal**: Serve 13-week curriculum with Docusaurus, enable browsing/reading chapters <3s

### Docusaurus Frontend Setup

- [ ] [T021] [P] [US1] Install Docusaurus 3.0 with TypeScript template (file: `frontend/docusaurus.config.ts`)
- [ ] [T022] [P] [US1] Configure sidebar navigation for 4 modules (file: `frontend/sidebars.ts`)
- [ ] [T023] [P] [US1] Customize Docusaurus theme with robotics color scheme (file: `frontend/src/css/custom.css`)
- [ ] [T024] [P] [US1] Create homepage with curriculum overview (file: `frontend/src/pages/index.tsx`)

### Content Structure & MDX

- [ ] [T025] [P] [US1] Create module directory structure: module-1/ through module-4/ (directories: `frontend/docs/module-{1,2,3,4}/`)
- [ ] [T026] [P] [US1] Create intro.md with course introduction (file: `frontend/docs/intro.md`)
- [ ] [T027] [US1] Write Module 1 Week 1: ROS 2 Introduction with MDX frontmatter (file: `frontend/docs/module-1/week-1-introduction.md`)
- [ ] [T028] [US1] Write Module 1 Week 2: Nodes and Topics (file: `frontend/docs/module-1/week-2-nodes-topics.md`)
- [ ] [T029] [US1] Write Module 1 Week 3: Services and Actions (file: `frontend/docs/module-1/week-3-services-actions.md`)
- [ ] [T030] [US1] Write Module 1 Week 4: TF and URDF (file: `frontend/docs/module-1/week-4-tf-urdf.md`)

### Interactive Components

- [ ] [T031] [P] [US1] Create HardwareTable component displaying Jetson/RealSense specs (file: `frontend/src/components/mdx/HardwareTable.tsx`)
- [ ] [T032] [P] [US1] Create InteractiveCodeBlock with copy-to-clipboard (file: `frontend/src/components/mdx/InteractiveCodeBlock.tsx`)
- [ ] [T033] [P] [US1] Create DiagramViewer for architecture diagrams (file: `frontend/src/components/mdx/DiagramViewer.tsx`)

### Performance & Deployment

- [ ] [T034] [US1] Optimize build: enable minification, lazy load images (file: `frontend/docusaurus.config.ts`)
- [ ] [T035] [US1] Deploy frontend to Vercel, verify <3s page load (external: Vercel)

**Independent Test**: Navigate to homepage, browse to module-1/week-1-introduction, verify page loads in <3 seconds, test on mobile 320px width

---

## Phase 4: User Story 2 - RAG Chatbot (P2)

**Goal**: Implement RAG chatbot with dual retrieval modes, >90% accuracy, <2s response, zero hallucinations

### Vector Database Indexing

- [ ] [T036] [US2] Create content indexing script with 500-token chunking (file: `backend/scripts/index_content.py`)
- [ ] [T037] [US2] Index Module 1 content into Qdrant `book_content_en` collection (command: `python scripts/index_content.py`)
- [ ] [T038] [US2] Verify 100+ chunks indexed with metadata (chapter_id, difficulty, hardware_requirement) (script: `backend/scripts/check_qdrant.py`)

### RAG Service Implementation

- [ ] [T039] [P] [US2] Implement RAG service with semantic search (file: `backend/src/services/rag.py`)
- [ ] [T040] [P] [US2] Create OpenAI Agents SDK chat agent with system prompt (file: `backend/src/agents/chat_agent.py`)
- [ ] [T041] [P] [US2] Create custom tool: navigate_to_section for book navigation (file: `backend/src/agents/tools.py`)

### API Endpoints

- [ ] [T042] [P] [US2] Create POST `/api/chat` endpoint for full-book context (file: `backend/src/api/chat.py`)
- [ ] [T043] [P] [US2] Create POST `/api/chat/selective` endpoint for selected-text only (file: `backend/src/api/chat.py`)
- [ ] [T044] [P] [US2] Implement response caching in Redis for repeated queries (file: `backend/src/services/rag.py`)

### Hallucination Testing (Constitutional Critical)

- [ ] [T045] [US2] Implement hallucination test suite with 6 critical tests (file: `backend/tests/integration/test_hallucination.py`)
- [ ] [T046] [US2] Test 1-3: Ungrounded questions, misleading premises, citation verification (file: `backend/tests/integration/test_hallucination.py`)
- [ ] [T047] [US2] Test 4-6: Token limit handling, selective context accuracy, confidence thresholds (file: `backend/tests/integration/test_hallucination.py`)

**Test**: Run `pytest backend/tests/integration/test_hallucination.py -v` - MUST PASS all 6 tests (zero tolerance)

### Frontend Integration

- [ ] [T048] [P] [US2] Create ChatbotWidget component with message history (file: `frontend/src/components/chat/ChatbotWidget.tsx`)
- [ ] [T049] [P] [US2] Create ChatbotToggle button for Chapter Action Bar (file: `frontend/src/components/buttons/ChatbotToggle.tsx`)
- [ ] [T050] [P] [US2] Create ContextModeToggle: full-book vs selective (file: `frontend/src/components/chat/ContextModeToggle.tsx`)
- [ ] [T051] [US2] Integrate chatbot into Docusaurus theme DocItem wrapper (file: `frontend/src/theme/DocItem/index.tsx`)

**Independent Test**: Open chatbot, ask 10 test questions (5 in-scope, 5 out-of-scope), verify >90% accuracy on in-scope, confirm "I don't have information" for out-of-scope, measure <2s response time

---

## Phase 5: User Story 3 - Auth & Profile (P3)

**Goal**: Enable account creation with hardware/skill profiling questionnaire

### Signup Flow

- [ ] [T052] [P] [US3] Create SignupForm component with email/password validation (file: `frontend/src/components/auth/SignupForm.tsx`)
- [ ] [T053] [P] [US3] Create ProfileQuestionnaire component with 3 questions (file: `frontend/src/components/auth/ProfileQuestionnaire.tsx`)
- [ ] [T054] [P] [US3] Create signup page integrating form + questionnaire (file: `frontend/src/pages/signup.tsx`)

### User Profile Management

- [ ] [T055] [P] [US3] Extend User model with hardware_profile, learning_goals fields (file: `backend/src/db/models.py`)
- [ ] [T056] [P] [US3] Create migration for profile fields (file: `backend/src/db/migrations/versions/002_add_profile.py`)
- [ ] [T057] [P] [US3] Create PATCH `/api/users/me` endpoint for profile updates (file: `backend/src/api/users.py`)

### Authentication UI

- [ ] [T058] [P] [US3] Create UserAuthButton for navbar (login/signup/profile menu) (file: `frontend/src/components/auth/UserAuthButton.tsx`)
- [ ] [T059] [P] [US3] Create signin page (file: `frontend/src/pages/signin.tsx`)
- [ ] [T060] [P] [US3] Create user dashboard showing progress (file: `frontend/src/pages/dashboard.tsx`)

**Independent Test**: Complete signup with email/password, fill questionnaire (hardware=Jetson, level=Beginner, experience=None), verify profile stored in database, login, access dashboard

---

## Phase 6: User Story 4 - Personalization (P4)

**Goal**: Adapt chapter content based on user profile (hardware, skill level)

### Personalization Engine

- [ ] [T061] [P] [US4] Create personalization service with rule engine (file: `backend/src/services/personalization.py`)
- [ ] [T062] [P] [US4] Define personalization rules: hardware access → show/hide sections (file: `backend/src/services/personalization.py`)
- [ ] [T063] [P] [US4] Define personalization rules: skill level → code complexity (file: `backend/src/services/personalization.py`)
- [ ] [T064] [P] [US4] Create POST `/api/content/personalize` endpoint (file: `backend/src/api/content.py`)

### Frontend Personalization

- [ ] [T065] [P] [US4] Create PersonalizationButton for Chapter Action Bar (file: `frontend/src/components/buttons/PersonalizationButton.tsx`)
- [ ] [T066] [P] [US4] Create ContentAdapter component applying show/hide rules (file: `frontend/src/components/personalization/ContentAdapter.tsx`)
- [ ] [T067] [P] [US4] Create PersonalizationModal displaying adapted content (file: `frontend/src/components/personalization/PersonalizationModal.tsx`)
- [ ] [T068] [US4] Cache personalization rules in localStorage for repeat visits (file: `frontend/src/components/personalization/PersonalizationButton.tsx`)

**Independent Test**: Login as "Beginner + No Hardware" user, click "Personalize This Chapter", verify cloud simulation shown, Jetson instructions hidden, code examples simplified

---

## Phase 7: User Story 5 - Translation (P5)

**Goal**: Translate chapters to Urdu while preserving code blocks and technical terms

### Translation Service

- [ ] [T069] [P] [US5] Create Google Translate API client with authentication (file: `backend/src/services/translation.py`)
- [ ] [T070] [P] [US5] Define technical glossary: ROS 2, Gazebo, Isaac Sim with Urdu transliterations (file: `backend/src/services/translation.py`)
- [ ] [T071] [P] [US5] Implement translation service: detect code blocks, preserve unchanged (file: `backend/src/services/translation.py`)
- [ ] [T072] [P] [US5] Create POST `/api/translate` endpoint with caching (file: `backend/src/api/content.py`)

### Frontend Translation

- [ ] [T073] [P] [US5] Create TranslationButton for Chapter Action Bar (file: `frontend/src/components/buttons/TranslationButton.tsx`)
- [ ] [T074] [P] [US5] Create translation state management with React Context (file: `frontend/src/contexts/TranslationContext.tsx`)
- [ ] [T075] [US5] Integrate translation toggle into DocItem theme (file: `frontend/src/theme/DocItem/index.tsx`)

### Urdu Vector Collection

- [ ] [T076] [US5] Create Qdrant collection `book_content_ur` (external: Qdrant Cloud)
- [ ] [T077] [US5] Translate and index Module 1 content into `book_content_ur` (script: `backend/scripts/translate_and_index.py`)

**Independent Test**: Click "Translate to Urdu", verify text translates, code blocks unchanged, technical terms show English in parentheses

---

## Phase 8: User Story 6 - AI Agents (P6)

**Goal**: Enable content creators to generate MDX chapters and implement features via agents

### Agent System Infrastructure

- [ ] [T078] [P] [US6] Create agent registry system (file: `.specify/agents/registry.json`)
- [ ] [T079] [P] [US6] Create `/agents` slash command for listing/invoking agents (file: `.claude/commands/agents.md`)

### DocusaurusContentAgent

- [ ] [T080] [P] [US6] Write DocusaurusContentAgent system prompt (file: `.specify/agents/docusaurus-spec/prompt.md`)
- [ ] [T081] [P] [US6] Define agent skills: mdx-generation, docusaurus-formatting (file: `.specify/agents/docusaurus-spec/skills.md`)
- [ ] [T082] [P] [US6] Create example: Chapter spec → MDX output (file: `.specify/agents/docusaurus-spec/examples/week-1-ros2.md`)
- [ ] [T083] [US6] Test agent: generate Module 2 Week 5 Gazebo chapter (command: `/agents docusaurus-spec "Create Week 5: Gazebo Basics"`)

### RAGIntegrationAgent

- [ ] [T084] [P] [US6] Write RAGIntegrationAgent system prompt (file: `.specify/agents/rag-integration/prompt.md`)
- [ ] [T085] [P] [US6] Define hallucination test scenarios (file: `.specify/agents/rag-integration/tests.md`)
- [ ] [T086] [US6] Test agent: implement retrieval optimization (command: `/agents rag-integration "Optimize semantic search"`)

### PersonalizationAgent

- [ ] [T087] [P] [US6] Write PersonalizationAgent system prompt (file: `.specify/agents/personalization-spec/prompt.md`)
- [ ] [T088] [P] [US6] Define personalization content rules (file: `.specify/agents/personalization-spec/content_rules.md`)

### UIComponentAgent

- [ ] [T089] [P] [US6] Write UIComponentAgent system prompt (file: `.specify/agents/ui-component/prompt.md`)
- [ ] [T090] [P] [US6] Document button design system (file: `.specify/agents/ui-component/design_system.md`)

### DevOpsAgent

- [ ] [T091] [P] [US6] Write DevOpsAgent system prompt (file: `.specify/agents/devops/prompt.md`)
- [ ] [T092] [P] [US6] Create deployment config templates (file: `.specify/agents/devops/deployment_config.md`)

**Independent Test**: Invoke DocusaurusContentAgent, verify valid MDX generated with frontmatter, code examples, interactive components

---

## Phase 9: Polish & Cross-Cutting Concerns

**Goal**: Optimize performance, implement monitoring, complete testing, final deployment

### Button System Implementation

- [ ] [T093] [P] [Polish] Install shadcn/ui components (Button, Modal) (file: `frontend/src/components/common/Button.tsx`)
- [ ] [T094] [P] [Polish] Configure Framer Motion for button animations (file: `frontend/package.json`)
- [ ] [T095] [P] [Polish] Implement BookmarkButton for Chapter Action Bar (file: `frontend/src/components/buttons/BookmarkButton.tsx`)
- [ ] [T096] [P] [Polish] Implement TextSelectionToolbar for inline actions (file: `frontend/src/components/buttons/TextSelectionToolbar.tsx`)
- [ ] [T097] [P] [Polish] Implement CodeBlockActions (copy, run, expand) (file: `frontend/src/components/buttons/CodeBlockActions.tsx`)
- [ ] [T098] [P] [Polish] Implement HardwareToggleButtons (cloud/Jetson/robot) (file: `frontend/src/components/buttons/HardwareToggleButtons.tsx`)

### Progress Tracking

- [ ] [T099] [P] [Polish] Create UserProgress model and migration (file: `backend/src/db/models.py`)
- [ ] [T100] [P] [Polish] Create POST `/api/progress` endpoint (file: `backend/src/api/progress.py`)
- [ ] [T101] [P] [Polish] Create ProgressTracker component showing chapter completion (file: `frontend/src/components/common/ProgressTracker.tsx`)

### Bookmark System

- [ ] [T102] [P] [Polish] Create Bookmark model and migration (file: `backend/src/db/models.py`)
- [ ] [T103] [P] [Polish] Create POST `/api/bookmarks` endpoint (file: `backend/src/api/bookmarks.py`)
- [ ] [T104] [P] [Polish] Integrate BookmarkButton with backend (file: `frontend/src/components/buttons/BookmarkButton.tsx`)

### Testing & QA

- [ ] [T105] [P] [Polish] Write frontend unit tests for components (file: `frontend/src/__tests__/`)
- [ ] [T106] [P] [Polish] Write Playwright E2E tests for user flows (file: `frontend/e2e/`)
- [ ] [T107] [P] [Polish] Write backend unit tests for services (file: `backend/tests/unit/`)
- [ ] [T108] [Polish] Run full test suite: `npm test && pytest` - verify all pass

### Performance Optimization

- [ ] [T109] [P] [Polish] Optimize images: convert to WebP, implement lazy loading (file: `frontend/static/img/`)
- [ ] [T110] [P] [Polish] Implement code splitting for React components (file: `frontend/docusaurus.config.ts`)
- [ ] [T111] [P] [Polish] Configure Redis caching for API responses (file: `backend/src/core/redis_client.py`)
- [ ] [T112] [Polish] Run Lighthouse audit: verify >90 score (command: `lighthouse`)

### Deployment & Monitoring

- [ ] [T113] [P] [Polish] Create GitHub Actions workflow for frontend deployment (file: `.github/workflows/frontend-deploy.yml`)
- [ ] [T114] [P] [Polish] Create GitHub Actions workflow for backend deployment (file: `.github/workflows/backend-deploy.yml`)
- [ ] [T115] [P] [Polish] Create CI test workflow running hallucination tests (file: `.github/workflows/tests.yml`)
- [ ] [T116] [P] [Polish] Set up Sentry for error tracking (external: Sentry)
- [ ] [T117] [P] [Polish] Set up Vercel Analytics for performance monitoring (external: Vercel)

### Documentation

- [ ] [T118] [P] [Polish] Update README.md with setup instructions (file: `README.md`)
- [ ] [T119] [P] [Polish] Create CONTRIBUTING.md for contributors (file: `CONTRIBUTING.md`)
- [ ] [T120] [Polish] Verify quickstart.md is accurate and reproducible (file: `specs/001-physical-ai-platform/quickstart.md`)

**Final Test**: Run full acceptance suite (all 6 user story independent tests), verify all success criteria met

---

## Dependency Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
    ├──→ Phase 3 (US1: Content Access)
    ├──→ Phase 4 (US2: RAG Chatbot) [depends on Phase 3 for content]
    ├──→ Phase 5 (US3: Auth & Profile)
    ├──→ Phase 6 (US4: Personalization) [depends on Phase 5 for auth]
    ├──→ Phase 7 (US5: Translation)
    └──→ Phase 8 (US6: AI Agents)
    ↓
Phase 9 (Polish) [depends on all user story phases]
```

**Parallel Execution Opportunities**:
- Within Phase 2: T012-T016 (backend services) can run parallel
- Within Phase 3: T021-T024 (Docusaurus setup) can run parallel
- Within Phase 4: T039-T041 (RAG service) can run parallel
- Within Phase 5: T052-T054 (signup UI) can run parallel
- All tasks marked [P] within their phase are parallelizable

---

## Acceptance Criteria

Each task must satisfy:
- ✅ Code written and committed to `001-physical-ai-platform` branch
- ✅ Tests passing (unit tests for [P] tasks, integration tests for phase completion)
- ✅ Documentation updated (inline comments, README sections)
- ✅ Linting passed (eslint for TypeScript, ruff for Python)
- ✅ No unresolved TODOs or placeholder code
- ✅ Constitutional compliance verified (especially hallucination tests for US2)

**Phase Completion Gates**:
- Phase 1: Repository structure exists, environment variables configured
- Phase 2: All databases connected, migrations run, auth endpoints functional
- Phase 3: Module 1 content accessible, <3s page load verified
- Phase 4: Hallucination tests PASS (zero tolerance), chatbot functional
- Phase 5: User can signup, login, access dashboard
- Phase 6: Personalization rules apply correctly to test profiles
- Phase 7: Urdu translation preserves code integrity
- Phase 8: At least 2 agents (Docusaurus + RAG) operational
- Phase 9: All 6 user story independent tests PASS

---

## Implementation Notes

**Critical Path**: Phase 1 → Phase 2 → Phase 3 (content) → Phase 4 (chatbot hallucination tests)

**Hallucination Testing (Constitutional Critical)**:
- Task T045-T047 MUST PASS before US2 considered complete
- Zero-tolerance policy: even 1 hallucination fails the entire feature
- Test suite runs in CI for every commit to backend/

**Performance Benchmarks**:
- Page load: measured with Lighthouse, <3s on 5 Mbps connection
- Chatbot response: p95 latency logged in Redis, <2s threshold
- Concurrent users: load test with 1000 simulated users, monitor response times

**Agent Invocation Workflow**:
1. Content creator defines chapter specification (learning objectives, topics)
2. Invoke: `/agents docusaurus-spec "Create Week X: Topic"`
3. Agent generates MDX with frontmatter, code examples, interactive components
4. Creator reviews output, makes edits if needed
5. Run indexing script to add content to Qdrant
6. PHR automatically created documenting the invocation

**Deployment Strategy**:
- Frontend (Vercel): Auto-deploy on push to main, preview deployments for PRs
- Backend (Railway): Auto-deploy from Dockerfile, environment variables via dashboard
- Database migrations: Manual `alembic upgrade head` on Railway console before deployment

**Cost Management**:
- Monitor Qdrant storage usage (max 1GB free tier), compress embeddings if needed
- Monitor OpenAI API costs via usage dashboard, implement rate limiting if budget exceeded
- Cache translations aggressively to minimize Google Translate API calls
