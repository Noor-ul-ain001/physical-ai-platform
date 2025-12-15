# RAG Integration Agent - Skills

This document defines the capabilities (skills) of the RAG Integration Agent with input/output contracts and test scenarios.

## Skill 1: vector-embedding

**Description**: Generate and manage vector embeddings for text chunks using efficient embedding models.

**Input Contract**:
```typescript
{
  textChunks: Array<{
    id: string;
    content: string;
    metadata: {
      source: string;
      pageTitle?: string;
      section?: string;
    };
  }>;
  embeddingModel: 'text-embedding-3-small' | 'text-embedding-ada-002' | string;
  batchSize?: number;
}
```

**Output Contract**:
```typescript
{
  embeddings: Array<{
    id: string;
    vector: number[];  // Embedding vector
    dimension: number;
  }>;
  stats: {
    totalChunks: number;
    totalTokens: number;
    estimatedCost: number;
    processingTime: number;
  };
}
```

**Test Cases**:
- ✅ **Happy Path**: 100 text chunks → Embeddings generated, dimensions match model spec
- ⚠️ **Edge Case**: Chunk exceeds token limit → Auto-split or warn
- ❌ **Failure**: API key invalid → Clear error with setup instructions

---

## Skill 2: retrieval-pipeline

**Description**: Implement semantic search with relevance scoring and result ranking.

**Input Contract**:
```typescript
{
  query: string;
  queryEmbedding?: number[];  // Pre-computed or will generate
  collectionName: string;
  topK: number;
  similarityThreshold: number;
  filters?: {
    source?: string[];
    pageTitle?: string[];
    dateRange?: {start: string, end: string};
  };
}
```

**Output Contract**:
```typescript
{
  results: Array<{
    id: string;
    content: string;
    metadata: Record<string, any>;
    score: number;  // Similarity score
    rank: number;
  }>;
  retrievalTime: number;  // Milliseconds
  totalMatches: number;
}
```

**Test Cases**:
- ✅ **Happy Path**: Query "ROS 2 nodes" → Retrieve relevant chunks, score > threshold
- ⚠️ **Edge Case**: No results above threshold → Return empty, suggest query refinement
- ❌ **Failure**: Qdrant connection timeout → Retry with exponential backoff

---

## Skill 3: qdrant-integration

**Description**: Configure Qdrant Cloud, manage collections, optimize vector operations.

**Input Contract**:
```typescript
{
  operation: 'create-collection' | 'index' | 'search' | 'delete';
  collectionConfig?: {
    name: string;
    vectorSize: number;
    distance: 'Cosine' | 'Euclidean' | 'Dot';
    onDiskPayload?: boolean;
  };
  data?: Array<{
    id: string;
    vector: number[];
    payload: Record<string, any>;
  }>;
}
```

**Output Contract**:
```typescript
{
  success: boolean;
  collectionInfo?: {
    name: string;
    vectorCount: number;
    indexedVectorsCount: number;
    pointsCount: number;
    status: 'green' | 'yellow' | 'red';
  };
  error?: string;
}
```

**Test Cases**:
- ✅ **Happy Path**: Create collection with 1536 dimensions → Collection created successfully
- ⚠️ **Edge Case**: Collection already exists → Skip or update
- ❌ **Failure**: Free tier limit exceeded → Error with upgrade guidance

---

## Skill 4: fastapi-endpoints

**Description**: Build REST API endpoints for chat, retrieval, and context management.

**Input Contract**:
```typescript
{
  endpoints: Array<{
    path: string;
    method: 'GET' | 'POST' | 'PUT' | 'DELETE';
    requestModel: string;  // Pydantic model name
    responseModel: string;
    handler: string;  // Function name
    description: string;
  }>;
  middleware?: string[];  // CORS, auth, logging
  errorHandling: boolean;
}
```

**Output Contract**:
```typescript
{
  apiCode: string;  // Complete FastAPI application code
  models: string;  // Pydantic models code
  routes: string[];  // List of created routes
  openAPISpec: object;  // Auto-generated OpenAPI schema
}
```

**Test Cases**:
- ✅ **Happy Path**: Define /api/chat endpoint → FastAPI code with proper typing, error handling
- ⚠️ **Edge Case**: Duplicate endpoint paths → Warn and suggest resolution
- ❌ **Failure**: Invalid Pydantic model → Validation error with fix suggestion

---

## Skill 5: hallucination-testing

**Description**: Create test suites that detect and prevent hallucinations in RAG responses.

**Input Contract**:
```typescript
{
  corpus: string;  // Path to indexed content
  testScenarios: Array<{
    type: 'ungrounded-question' | 'misleading-question' | 'citation-check';
    question: string;
    expectedBehavior: string;
  }>;
  tolerance: 'zero' | 'low';
}
```

**Output Contract**:
```typescript
{
  testCode: string;  // pytest test suite code
  testCases: Array<{
    name: string;
    description: string;
    critical: boolean;
  }>;
  executionInstructions: string;
}
```

**Test Cases**:
- ✅ **Happy Path**: Generate hallucination tests → Comprehensive pytest suite with assertions
- ⚠️ **Edge Case**: Corpus empty → Tests check for proper "no data" handling
- ❌ **Failure**: Test suite has syntax errors → Validation error

---

## Skill Composition Examples

### Example 1: Complete RAG Pipeline Setup

```
vector-embedding (index content)
  → qdrant-integration (create collection, store vectors)
  → fastapi-endpoints (create chat API)
  → hallucination-testing (validate system)
  → Deployed RAG system
```

### Example 2: Query Processing Flow

```
fastapi-endpoints (receive query)
  → vector-embedding (embed query)
  → retrieval-pipeline (search Qdrant)
  → Citation generation (extract sources)
  → Response assembly
```

### Example 3: Quality Assurance Workflow

```
hallucination-testing (run test suite)
  → retrieval-pipeline (verify relevance)
  → Citation validation
  → Performance benchmarking
  → Green light for deployment
```

## Skill Dependencies

- **vector-embedding** → No dependencies (foundational)
- **retrieval-pipeline** → Requires `vector-embedding` + `qdrant-integration`
- **qdrant-integration** → No dependencies (foundational)
- **fastapi-endpoints** → Requires `retrieval-pipeline` for handlers
- **hallucination-testing** → Requires complete RAG pipeline to test

## Quality Metrics

Each skill should achieve:
- **Correctness**: 100% (no hallucinations, accurate retrievals)
- **Performance**: p95 < 2s for retrieval-pipeline, < 500ms for vector operations
- **Constitutional Alignment**: 100% (Article III compliance)
- **Test Coverage**: > 90% for all skills
- **Security**: No injection vulnerabilities, API key protection

## Advanced Techniques (Optional)

- **Re-ranking**: Post-retrieval re-scoring using cross-encoders
- **Hybrid search**: Combine vector search with keyword matching
- **Query expansion**: Rephrase user query for better retrieval
- **Context compression**: Reduce retrieved text while preserving meaning
- **Multi-hop reasoning**: Chain retrievals for complex questions

## Future Skills (Planned)

- **adaptive-chunking**: Dynamic chunk sizes based on content structure
- **semantic-cache**: Cache embeddings for frequently accessed content
- **answer-synthesis**: Advanced LLM-based answer generation with citations
- **feedback-loop**: Incorporate user feedback to improve retrieval
- **multimodal-retrieval**: Support images, diagrams in addition to text
