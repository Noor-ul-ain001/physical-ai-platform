# Tasks: AI-Driven Development Book & RAG Chatbot

**Input**: Design documents from `specs/001-rag-chatbot-book/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/openapi.yml

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both frontend and backend.

- [x] T001 Create the monorepo structure with `frontend` and `backend` directories.
- [x] T002 [P] In the `frontend` directory, initialize a new Docusaurus site using the classic template (`npx create-docusaurus@latest frontend classic`).
- [x] T003 [P] In the `backend` directory, set up a new Python project with a virtual environment.
- [x] T004 [P] In `backend`, install FastAPI, Uvicorn, and other core Python dependencies (`pip install fastapi uvicorn python-dotenv`).
- [x] T005 [P] Configure basic linting and formatting for both Python (`black`, `ruff`) and JavaScript (`eslint`, `prettier`).

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [x] T006 [Backend] Set up a free-tier account on Qdrant Cloud and obtain API key and URL. Store these in a `.env` file in `backend/.env`.
- [x] T007 [Backend] Implement the Qdrant client connection in `backend/src/core/qdrant_client.py`.
- [x] T008 [Backend] Create an initial version of the embedding script in `backend/src/core/embed.py`. This script should read a sample markdown file, chunk it, generate embeddings (using OpenAI), and upload the vectors to Qdrant.
- [x] T009 [Backend] Add basic CORS middleware to the FastAPI app in `backend/src/main.py` to allow requests from the frontend's future GitHub Pages URL.
- [x] T010 [Frontend] Create placeholder book content (e.g., `chapter1.md`, `chapter2.md`) in the `frontend/docs/` directory.
- [x] T011 [Frontend] Configure `frontend/docusaurus.config.js` for deployment to GitHub Pages, setting the correct `organizationName`, `projectName`, `deploymentBranch`, and `trailingSlash`.

**Checkpoint**: Foundation ready. The backend can connect to Qdrant and embed content. The frontend has placeholder content and is configured for deployment.

---

## Phase 3: User Story 1 - Read the Book (Priority: P1) 🎯 MVP

**Goal**: A user can visit the published website and read the book's content.

**Independent Test**: The Docusaurus website is successfully deployed and publicly accessible on GitHub Pages. Navigation between chapters works correctly.

- [x] T012 [US1] [Frontend] Populate `frontend/docs/` with at least two chapters of polished, sample content.
- [x] T013 [US1] [Frontend] Configure the sidebar in `frontend/sidebars.js` to correctly order and display the chapters.
- [x] T014 [US1] [Frontend] Run the `docusaurus deploy` command from the `frontend` directory to publish the initial version of the book to GitHub Pages.
- [x] T015 [US1] [Frontend] Verify that the site is live and that all content renders correctly.

**Checkpoint**: At this point, the book is a functional, readable website. This is the first deliverable milestone (MVP).

---

## Phase 4: User Story 2 - Chatbot Q&A (Full Context) (Priority: P2)

**Goal**: Implement the primary chatbot interface that answers questions based on the full book content.

**Independent Test**: A user can ask a question on the chatbot page and receive an accurate, context-aware answer from the backend.

- [x] T016 [US2] [Backend] Implement the `/chat` endpoint in `backend/src/api/chat.py` as defined in `contracts/openapi.yml`.
- [x] T017 [US2] [Backend] Implement the core RAG service in `backend/src/services/rag.py`. This service will take a query, create an embedding, search Qdrant for relevant text chunks, and use the OpenAI Agents SDK to generate a response from the retrieved context.
- [x] T018 [US2] [Frontend] Create a dedicated chatbot page at `frontend/src/pages/chatbot.js`.
- [x] T019 [US2] [Frontend] Create the main chat UI as a React component in `frontend/src/components/ChatInterface.js`. It should include a message display area, a text input field, and a submit button.
- [x] T020 [US2] [Frontend] In `ChatInterface.js`, add the client-side logic to make a `POST` request to the backend's `/chat` endpoint with the user's query.
- [x] T021 [US2] [Frontend] Display the answer from the API response in the chat message area.

**Checkpoint**: The chatbot is now integrated and functional for full-context questions.

---

## Phase 5: User Story 3 - Chatbot Q&A (Selected Context) (Priority: P3)

**Goal**: Allow users to ask questions about a specific piece of selected text.

**Independent Test**: A user can select text on any page, click a button, which opens the chat modal, and ask a question that is answered *only* from the selected text.

- [x] T022 [US3] [Backend] Implement the `/chat-with-selection` endpoint in `backend/src/api/chat.py`. The service layer for this should *not* call Qdrant, but instead feed the user-provided context directly to the LLM.
- [c] T023 [US3] [Frontend] Use Docusaurus swizzling or custom components to inject a script that adds an "Ask about this selection" button, which appears when a user highlights text on any documentation page. (Cancelled by user)
- [c] T024 [US3] [Frontend] Implement a chat modal that opens when the "Ask" button is clicked, pre-filling the context. (Cancelled - dependent on T023)
- [c] T025 [US3] [Frontend] The chat modal will call the `/chat-with-selection` endpoint and display the result. (Cancelled - dependent on T023)

**Checkpoint**: The selected-text feature is complete, providing a focused Q&A experience.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [x] T026 [P] [Frontend] Refine the styling of the chat interface and the "Ask about selection" button.
- [x] T027 [P] [Backend] Add robust error handling and logging to all API endpoints.
- [x] T028 [P] [Backend] Enhance the `embed.py` script to automatically detect content changes and update the Qdrant index.
- [x] T029 [P] [Frontend] Ensure the chat interface is accessible and mobile-friendly.

---

## Dependencies & Execution Order

- **Setup (Phase 1)** -> **Foundational (Phase 2)** -> **User Stories (Phase 3+)**
- **User Story 1 (P1)** is the MVP and can be delivered independently after Phase 2.
- **User Story 2 (P2)** depends on Phase 2.
- **User Story 3 (P3)** depends on Phase 2 for the backend logic and Phase 4 for the chat UI components it will reuse.

## Implementation Strategy

1.  **MVP First**: Complete all tasks for **Phase 1, 2, and 3 (User Story 1)**. This delivers a readable, deployed book, which is the core product.
2.  **Incremental Delivery**:
    -   Next, complete **Phase 4 (User Story 2)** to add the main chatbot functionality.
    -   Then, complete **Phase 5 (User Story 3)** to add the enhanced selection feature.
    -   Finally, address **Polish** tasks as time permits.
