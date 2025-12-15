# Data Model: Physical AI & Humanoid Robotics Educational Platform

**Feature**: 001-physical-ai-platform
**Date**: 2025-12-15

## Entity Relationship Overview

```
User (1) ----< (M) UserProgress
User (1) ----< (M) Bookmark
User (1) ----< (M) Highlight
User (1) ----< (M) ChatSession
ChatSession (1) ----< (M) ChatMessage
```

## Core Entities

### User

Represents a learner or content consumer with authentication and profile data.

**Fields**:
- `id` (UUID, PK): Unique identifier
- `email` (String, Unique, Indexed): User email for authentication
- `password_hash` (String): bcrypt hashed password (12 rounds minimum)
- `created_at` (Timestamp): Account creation timestamp
- `last_login` (Timestamp, Nullable): Most recent login
- `email_verified` (Boolean): Email verification status
- `hardware_profile` (JSONB): {gpu, robotics_kit, experience_level}
- `learning_goals` (JSONB Array): ["ROS 2", "Simulation", ...]
- `accessibility_prefs` (JSONB): {font_size, reduced_animations, language}
- `is_active` (Boolean): Account status

**Indexes**:
- Primary: `id`
- Unique: `email`
- Composite: `(email, is_active)`

**Validation**:
- Email must match regex: `^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$`
- Password minimum 8 characters, 1 uppercase, 1 lowercase, 1 number
- `hardware_profile.gpu` enum: ["None", "Integrated", "RTX 3060", "RTX 4080", "A100"]
- `hardware_profile.robotics_kit` enum: ["None", "Jetson Orin", "Full Robot"]
- `hardware_profile.experience_level` enum: ["Beginner", "Intermediate", "Advanced"]

---

### UserProgress

Tracks chapter completion and engagement metrics per user.

**Fields**:
- `id` (UUID, PK)
- `user_id` (UUID, FK → User.id, Indexed)
- `chapter_id` (String, Indexed): e.g., "module-1/week-1-introduction"
- `completion_percentage` (Integer, 0-100): Progress within chapter
- `last_accessed` (Timestamp): Most recent view
- `time_spent_seconds` (Integer): Total time on chapter
- `exercises_completed` (JSONB Array): ["exercise-1", "exercise-2"]
- `quiz_scores` (JSONB): {quiz_id: score}

**Indexes**:
- Primary: `id`
- Composite: `(user_id, chapter_id)` UNIQUE
- Index: `last_accessed`

---

### Bookmark

User-saved chapters or sections for quick access.

**Fields**:
- `id` (UUID, PK)
- `user_id` (UUID, FK → User.id, Indexed)
- `chapter_id` (String)
- `section_id` (String, Nullable): Specific section within chapter
- `notes` (Text, Nullable): User annotation
- `created_at` (Timestamp)

**Indexes**:
- Primary: `id`
- Composite: `(user_id, chapter_id, section_id)` UNIQUE

---

### Highlight

Text selections saved by users with optional annotations.

**Fields**:
- `id` (UUID, PK)
- `user_id` (UUID, FK → User.id, Indexed)
- `chapter_id` (String, Indexed)
- `text_selection` (Text): Highlighted text content
- `start_offset` (Integer): Character offset in chapter
- `end_offset` (Integer)
- `annotation` (Text, Nullable): User note
- `created_at` (Timestamp)

**Indexes**:
- Primary: `id`
- Composite: `(user_id, chapter_id)`

---

### ChatSession

Conversation sessions with the RAG chatbot.

**Fields**:
- `id` (UUID, PK)
- `user_id` (UUID, FK → User.id, Nullable, Indexed): Null for anonymous
- `session_token` (String, Indexed): For anonymous session tracking
- `context_mode` (Enum): ["full_book", "selective"]
- `chapter_context` (String, Nullable): Current chapter if applicable
- `created_at` (Timestamp)
- `last_message_at` (Timestamp)
- `message_count` (Integer): Total messages in session

**Indexes**:
- Primary: `id`
- Index: `user_id`
- Index: `session_token`
- Index: `last_message_at` (for cleanup)

**TTL**: Anonymous sessions deleted after 7 days

---

### ChatMessage

Individual messages within a chat session.

**Fields**:
- `id` (UUID, PK)
- `session_id` (UUID, FK → ChatSession.id, Indexed)
- `role` (Enum): ["user", "assistant"]
- `content` (Text): Message text
- `citations` (JSONB Array, Nullable): [{source, excerpt, similarity}]
- `retrieval_context` (JSONB, Nullable): {chunks_retrieved, top_k, threshold}
- `confidence_score` (Float, Nullable): 0.0-1.0
- `response_time_ms` (Integer, Nullable): Generation latency
- `created_at` (Timestamp)

**Indexes**:
- Primary: `id`
- Foreign: `session_id`
- Index: `created_at`

---

## Supporting Data (Not in Relational DB)

### Vector Embeddings (Qdrant)

**Collection**: `book_content_en`
**Vectors**: 1536 dimensions (text-embedding-3-small)
**Payload**:
```json
{
  "chapter_id": "module-1/week-1-introduction",
  "section_id": "ros2-basics",
  "content": "...",
  "content_type": "text" | "code" | "hardware_spec",
  "difficulty_level": "beginner" | "intermediate" | "advanced",
  "hardware_requirement": "none" | "jetson" | "full_robot",
  "chunk_index": 0,
  "total_chunks": 10
}
```

**Collection**: `book_content_ur`
Same structure, Urdu translated content.

---

### Session Cache (Redis)

**Keys**:
- `session:{session_token}` → User session data (TTL: 7 days)
- `personalization:{user_id}:{chapter_id}` → Personalization rules (TTL: 24 hours)
- `translation:{chapter_id}:ur` → Cached Urdu translation (TTL: 30 days)
- `rate_limit:chat:{ip}` → Chat request count (TTL: 1 minute)

---

## State Transitions

### User Lifecycle
```
[Anonymous] → [Signup] → [Email Verify*] → [Profile Setup] → [Active]
                                                                  ↓
                                                            [Deactivated]
```
*Optional for MVP

### Chat Session Lifecycle
```
[Created] → [Active] → [Idle >1 hour] → [Archived]
                                              ↓
                                    [Deleted after 7 days for anonymous]
```

### Chapter Progress
```
[Not Started] → [In Progress 1-99%] → [Completed 100%]
                        ↓
                  [Reset to 0%]
```

---

## Migrations Strategy

Use Alembic for database schema versioning:

```bash
# Create initial migration
alembic revision --autogenerate -m "Initial schema"

# Apply migrations
alembic upgrade head
```

**Migration files**: `backend/src/db/migrations/versions/`

---

## Data Retention

- **User accounts**: Indefinite (unless deletion requested)
- **Chat sessions (authenticated)**: Indefinite
- **Chat sessions (anonymous)**: 7 days
- **User progress**: Indefinite
- **Bookmarks/Highlights**: Indefinite
- **Redis cache**: Varies by key (1 min to 30 days)
- **Qdrant vectors**: Indefinite (rebuild from source content)
