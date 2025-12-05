# Data Models for Gemini Robotics Textbook

**Input**: `plan.md` for feature `001-gemini-robotics-textbook`

This document outlines the data structures, database schemas, and component interfaces for the project.

## Component Interfaces

### Voice Chatbot Component
```typescript
interface GeminiVoiceChatbot {
  // Voice capture with color states
  startListening(): Promise<void>; // #B279A4 pulse
  stopListening(): void;
  processAudioStream(stream: MediaStream): void;

  // Gemini STT processing
  transcribeWithGemini(audioData: Blob): Promise<string>;
  handleStreamingTranscription(): void;

  // Gemini RAG processing
  generateGeminiResponse(query: string, context: string): Promise<string>;
  retrieveRelevantChunks(query: string): Promise<DocumentChunk[]>;

  // TTS synthesis
  synthesizeWithGemini(text: string): Promise<AudioBuffer>;
  playAudio(audioData: AudioBuffer): void;

  // State management with color coding
  connectionState: 'connected' | 'connecting' | 'error';
  isListening: boolean; // #B279A4 when true
  isSpeaking: boolean; // #79B2A4 when true
}
```

### Personalization Engine User Profile
```typescript
interface UserProfile {
  id: string;
  background: {
    software: 'beginner' | 'intermediate' | 'expert';
    hardware: 'novice' | 'experienced' | 'expert';
    aiKnowledge: number; // 1-10 scale
    roboticsExperience: number; // 1-10 scale
  };
  preferences: {
    learningStyle: 'visual' | 'auditory' | 'kinesthetic';
    contentDepth: 'basic' | 'standard' | 'advanced';
    voiceEnabled: boolean;
    language: 'en' | 'ur';
    theme: 'light' | 'dark';
  };
  progress: {
    completedModules: string[];
    currentModule: string;
    assessmentScores: Record<string, number>;
  };
}
```

## Database Schema

### Postgres (Neon)

```sql
-- Users table
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- User profiles with background assessment
CREATE TABLE user_profiles (
  user_id UUID PRIMARY KEY REFERENCES users(id),
  software_level VARCHAR(50) NOT NULL,
  hardware_level VARCHAR(50) NOT NULL,
  ai_knowledge INTEGER CHECK (ai_knowledge >= 1 AND ai_knowledge <= 10),
  robotics_experience INTEGER CHECK (robotics_experience >= 1 AND robotics_experience <= 10),
  learning_style VARCHAR(50),
  preferred_language VARCHAR(10) DEFAULT 'en',
  theme_preference VARCHAR(10) DEFAULT 'light',
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- User progress tracking
CREATE TABLE user_progress (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),
  module_id VARCHAR(100) NOT NULL,
  chapter_id VARCHAR(100) NOT NULL,
  completed BOOLEAN DEFAULT FALSE,
  score INTEGER,
  time_spent INTEGER, -- in seconds
  created_at TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(user_id, module_id, chapter_id)
);

-- Voice interactions for analytics
CREATE TABLE voice_interactions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),
  query_text TEXT,
  response_text TEXT,
  stt_confidence FLOAT,
  response_time INTEGER, -- milliseconds
  language VARCHAR(10),
  created_at TIMESTAMPTZ DEFAULT NOW()
);
```

### Vector Database (Qdrant)

```yaml
# Collection for storing textbook content embeddings
textbook_content:
  vectors:
    size: 768 # Gemini embedding size
    distance: Cosine
  payload:
    - module: string
    - chapter: string
    - content_type: string
    - language: string
    - difficulty: string
    - tags: string[]

# Collection for storing user query embeddings for analytics
user_queries:
  vectors:
    size: 768
    distance: Cosine
  payload:
    - user_id: string
    - query: string
    - response: string
    - timestamp: string
```
