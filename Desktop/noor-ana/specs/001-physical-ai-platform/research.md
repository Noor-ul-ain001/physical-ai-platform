# Research: Physical AI & Humanoid Robotics Educational Platform

**Feature**: 001-physical-ai-platform
**Date**: 2025-12-15
**Purpose**: Resolve technical unknowns and establish best practices for implementation

## Research Questions Resolved

### 1. Docusaurus + React 18 + TypeScript Integration

**Decision**: Use Docusaurus 3.0 with React 18 and TypeScript 5.3

**Rationale**:
- Docusaurus 3.0 natively supports React 18 and TypeScript
- MDX v3 integration allows seamless React component embedding in Markdown
- Built-in hot reload and fast refresh for development
- SSG (Static Site Generation) + optional SSR for dynamic features
- Plugin ecosystem for search, PWA, analytics

**Best Practices**:
- Use `.tsx` extension for React components, `.mdx` for content pages
- Separate presentational components (`src/components/`) from MDX content (`docs/`)
- Leverage Docusaurus theming via `src/theme/` for navbar/footer customization
- Use `@docusaurus/plugin-client-redirects` for URL migration if needed
- Configure `docusaurus.config.ts` with TypeScript for type safety

**Alternatives Considered**:
- Next.js + Contentlayer: More flexible but requires custom doc site scaffolding
- Gatsby: Older ecosystem, slower builds for large content sites
- VitePress: Vue-based, not React (incompatible with requirement)

**References**:
- https://docusaurus.io/docs/typescript-support
- https://docusaurus.io/docs/markdown-features/react

---

### 2. Button System Architecture (shadcn/ui + Framer Motion)

**Decision**: Use shadcn/ui for base components with Framer Motion for animations

**Rationale**:
- shadcn/ui provides copy-paste Radix UI primitives (accessible, customizable)
- Tailwind CSS for consistent styling matching robotic theme
- Framer Motion for smooth transitions (button hover, modal open, chatbot slide)
- No heavyweight dependency (components are copied into codebase, not NPM package)
- Easy to customize button variants (primary, secondary, ghost, destructive)

**Implementation Strategy**:
- Create base `Button` component in `src/components/common/Button.tsx`
- Define variants: `primary` (robotic blue), `secondary` (circuit green), `ghost`, `destructive`
- Use Framer Motion's `motion.button` for hover/tap animations
- Implement consistent sizing system (40px header, 36px action bar, 32px inline)
- Add loading states with spinner icon from Lucide React

**Best Practices**:
- Use `asChild` pattern from Radix for flexible composition
- Implement keyboard navigation (Tab, Enter, Escape)
- Add ARIA labels for all buttons
- Test with screen readers (NVDA, JAWS)
- Use `useReducedMotion` hook to respect user preferences

**Alternatives Considered**:
- Material-UI: Heavy bundle size, opinionated design system
- Chakra UI: Good accessibility but not Tailwind-based
- Custom from scratch: Reinventing the wheel for accessibility

**References**:
- https://ui.shadcn.com/docs/components/button
- https://www.framer.com/motion/introduction/

---

### 3. RAG System: OpenAI Agents SDK vs. LangChain

**Decision**: Use OpenAI Agents SDK with custom tools for book navigation

**Rationale**:
- OpenAI Agents SDK (2024 release) provides native function calling and streaming
- Simpler architecture than LangChain for single-purpose RAG system
- Direct integration with GPT-4 for generation, text-embedding-3-small for embeddings
- Custom tools for book-specific actions (navigate_to_section, get_code_example)
- Better cost control with explicit API calls vs. LangChain abstractions

**Architecture**:
```python
from openai_agents_sdk import Agent, Tool

# Define custom tools
navigate_tool = Tool(
    name="navigate_to_section",
    description="Find and retrieve specific book sections",
    function=navigate_to_section_impl
)

# Initialize agent
chat_agent = Agent(
    model="gpt-4-turbo-preview",
    tools=[navigate_tool],
    system_prompt="You are a robotics teaching assistant..."
)
```

**Best Practices**:
- Stream responses for perceived speed (<2s feels instant)
- Implement token counting to stay within limits
- Cache embeddings in Qdrant for repeated queries
- Use async/await throughout for non-blocking I/O
- Implement retry logic with exponential backoff for API failures

**Alternatives Considered**:
- LangChain: Feature-rich but complex for single-domain RAG
- LlamaIndex: Good for multi-source retrieval, overkill here
- Custom implementation: Reinventing robust orchestration logic

**References**:
- https://platform.openai.com/docs/assistants/overview
- https://cookbook.openai.com/examples/how_to_call_functions_with_chat_models

---

### 4. Qdrant vs. Pinecone vs. Weaviate for Vector Storage

**Decision**: Use Qdrant Cloud (Free Tier)

**Rationale**:
- Free tier: 1GB storage, 100k vectors (sufficient for MVP: ~52 chapters * 2k vectors/chapter = 104k)
- Native filtering support (metadata: chapter_id, difficulty_level, content_type)
- Hybrid search (combine vector similarity + keyword matching)
- Python SDK with async support (`qdrant-client[async]`)
- Self-hosting option for future scale-out

**Optimization Strategy**:
- Chunk size: 500 tokens (balances context vs. storage)
- Overlap: 50 tokens between chunks for continuity
- Embedding model: text-embedding-3-small (1536 dimensions, $0.02/1M tokens)
- Separate collections: `book_content_en`, `book_content_ur` for English/Urdu
- Metadata fields: `chapter_id`, `section_id`, `difficulty`, `hardware_type`

**Best Practices**:
- Use `scroll` API for bulk ingestion (not point-by-point inserts)
- Implement exponential backoff for rate limits
- Monitor storage usage via Qdrant dashboard
- Compress embeddings if approaching 100k limit (PCA dimensionality reduction)

**Alternatives Considered**:
- Pinecone: Paid-only, no free tier for production
- Weaviate: More complex setup, multi-tenancy overhead
- Postgres with pgvector: Limited to 2000 dimensions, slower at scale

**References**:
- https://qdrant.tech/documentation/quick-start/
- https://qdrant.tech/documentation/concepts/collections/

---

### 5. Better Auth vs. NextAuth vs. Auth.js

**Decision**: Use Better Auth with email/password + OAuth

**Rationale**:
- TypeScript-first with full type safety
- Extended schema support for custom user fields (hardware_profile, learning_goals)
- Session management with secure cookies (httpOnly, sameSite, signed)
- Built-in CSRF protection
- Easy OAuth integration (Google, GitHub) for future enhancement
- Active development, modern codebase

**Implementation**:
```typescript
// backend/src/auth/config.ts
import { BetterAuth } from 'better-auth';

export const auth = new BetterAuth({
  database: neonPool, // Postgres connection
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Optional for MVP
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Refresh daily
  },
  callbacks: {
    async signUp({ user }) {
      // Redirect to profile questionnaire
      return { redirectTo: '/profile-setup' };
    },
  },
});
```

**Best Practices**:
- Hash passwords with bcrypt (12 rounds minimum)
- Implement rate limiting on auth endpoints (5 attempts/min)
- Use HTTPS-only cookies in production
- Store session tokens in Redis for fast validation
- Implement "Remember Me" with longer-lived refresh tokens

**Alternatives Considered**:
- NextAuth: Coupled to Next.js, not ideal for Docusaurus
- Auth.js: Newer rebrand of NextAuth, similar coupling
- Passport.js: Lower-level, requires more custom code

**References**:
- https://better-auth.com/docs/introduction
- https://better-auth.com/docs/email-password

---

### 6. Google Translate API vs. DeepL for Urdu Translation

**Decision**: Use Google Cloud Translation API with custom glossary

**Rationale**:
- Strong Urdu support (one of 100+ supported languages)
- Custom glossary feature for technical terms (ROS → روبوٹ آپریٹنگ سسٹم)
- Pay-as-you-go pricing ($20/1M characters)
- Batch translation API for efficiency
- Neural Machine Translation (NMT) for context-aware translation

**Implementation Strategy**:
```python
from google.cloud import translate_v2 as translate

# Initialize client
translate_client = translate.Client()

# Custom glossary for technical terms
glossary = {
    "ROS 2": "ROS 2",  # Keep acronyms
    "Gazebo": "Gazebo",
    "Isaac Sim": "Isaac Sim",
    "node": "نوڈ (Node)",
    "topic": "ٹاپک (Topic)",
}

def translate_chapter(text: str) -> str:
    # Replace technical terms with glossary
    for en, ur in glossary.items():
        text = text.replace(en, f"{{KEEP:{en}}}")

    # Translate
    result = translate_client.translate(text, target_language='ur')

    # Restore glossary terms
    translated = result['translatedText']
    for en, ur in glossary.items():
        translated = translated.replace(f"{{KEEP:{en}}}", ur)

    return translated
```

**Best Practices**:
- Detect code blocks with regex, skip translation
- Cache translations in database to avoid re-translation costs
- Implement fallback to English if translation fails
- Use HTML preservation mode to maintain MDX structure
- Batch multiple paragraphs per API call for cost efficiency

**Alternatives Considered**:
- DeepL: Limited Urdu support (only European languages well-supported)
- Azure Translator: Similar to Google but weaker Urdu performance
- Manual translation: Not scalable for 52+ chapters

**References**:
- https://cloud.google.com/translate/docs/basic/translating-text
- https://cloud.google.com/translate/docs/advanced/glossary

---

### 7. Content Personalization Strategy

**Decision**: Client-side rendering with server-side profile hydration

**Rationale**:
- Docusaurus is SSG (Static Site Generation), can't personalize at build time
- Hybrid approach: Load canonical content statically, personalize dynamically
- Use React Context for user profile state
- Fetch personalization rules from FastAPI `/api/content/personalize`
- Apply rules client-side to show/hide sections, swap code examples

**Architecture**:
```typescript
// Frontend: PersonalizationContext.tsx
const PersonalizationContext = React.createContext({
  userProfile: null,
  personalizeContent: (content) => content,
});

// Component: PersonalizationButton.tsx
function PersonalizationButton() {
  const { personalizeContent } = usePersonalization();

  const handleClick = async () => {
    const profile = await fetch('/api/users/me').then(r => r.json());
    const rules = await fetch('/api/content/personalize', {
      body: JSON.stringify({ profile, chapterId: currentChapter }),
    }).then(r => r.json());

    applyPersonalizationRules(rules);
  };
}
```

**Personalization Rules**:
- **Hardware Access = "None"**: Hide Jetson setup, show cloud simulation
- **Programming Level = "Beginner"**: Add code comments, show simpler examples
- **Robotics Experience = "None"**: Surface prerequisite sections, add glossary

**Best Practices**:
- Cache personalization rules in localStorage for repeat visits
- Use CSS classes for show/hide (`.personalized-hide`, `.personalized-show`)
- Implement smooth transitions with Framer Motion
- Preserve canonical URL (personalization is view state, not separate pages)
- A/B test personalization effectiveness with analytics

**Alternatives Considered**:
- Server-side rendering: Not compatible with Docusaurus SSG
- Build-time personalization: Exponential page variants (not scalable)
- No personalization: Misses constitutional requirement (Article V)

---

### 8. Deployment Architecture (Vercel + Railway)

**Decision**: Vercel for frontend, Railway for backend, Neon/Qdrant/Upstash for data

**Rationale**:
- **Vercel**: Optimized for Docusaurus (SSG + edge functions), auto-scaling, global CDN
- **Railway**: FastAPI auto-deployment from GitHub, Dockerfile support, $5/month credit
- **Neon**: Serverless Postgres with 0.5GB free tier, auto-pause when idle
- **Qdrant Cloud**: 1GB free vector storage, managed service
- **Upstash Redis**: 10k commands/day free, global edge caching

**CI/CD Flow**:
```yaml
# .github/workflows/frontend-deploy.yml
name: Deploy Frontend
on:
  push:
    branches: [main]
    paths: ['frontend/**']

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
      - run: cd frontend && npm ci && npm run build
      - uses: amondnet/vercel-action@v25
        with:
          vercel-token: ${{ secrets.VERCEL_TOKEN }}
          vercel-project-id: ${{ secrets.VERCEL_PROJECT_ID }}
```

**Best Practices**:
- Use environment variables for secrets (DATABASE_URL, QDRANT_API_KEY)
- Implement health check endpoints (`/api/health`, `/health`)
- Enable auto-scaling triggers (CPU > 70%, memory > 80%)
- Set up Sentry for error tracking
- Use Vercel Analytics for frontend performance monitoring

**Alternatives Considered**:
- Netlify: Similar to Vercel but weaker Next.js/React optimization
- Render: Good Railway alternative but slower cold starts
- AWS (Amplify + Lambda): More complex setup, overkill for MVP

**References**:
- https://vercel.com/docs/frameworks/docusaurus
- https://railway.app/docs/develop/services

---

## Technology Stack Summary

| Layer | Technology | Version | Purpose |
|-------|-----------|---------|---------|
| **Frontend** | Docusaurus | 3.0 | Static site generation |
| | React | 18.2 | UI components |
| | TypeScript | 5.3 | Type safety |
| | shadcn/ui | latest | Component library |
| | Framer Motion | 11.x | Animations |
| **Backend** | FastAPI | 0.104+ | REST API |
| | Python | 3.11 | Runtime |
| | Pydantic | v2 | Data validation |
| | OpenAI Agents SDK | latest | RAG orchestration |
| **Databases** | Neon Postgres | Serverless | User data, sessions |
| | Qdrant Cloud | Free Tier | Vector embeddings |
| | Upstash Redis | Free Tier | Caching, sessions |
| **Auth** | Better Auth | latest | Authentication |
| **Translation** | Google Translate | v2 | Urdu translation |
| **Deployment** | Vercel | latest | Frontend hosting |
| | Railway | latest | Backend hosting |
| **Testing** | Jest | 29.x | Frontend unit tests |
| | pytest | 7.x | Backend unit tests |
| | Playwright | 1.40+ | E2E tests |

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Qdrant free tier exceeded | Monitor usage, implement aggressive chunking optimization, fallback to Postgres pgvector |
| Translation API costs | Cache all translations in database, batch translate during content creation not runtime |
| Hallucination in chatbot | Implement comprehensive test suite (6 tests), manual spot-checks, user feedback loop |
| Performance < 3s page load | Optimize images (WebP, lazy load), code split React components, edge caching |
| Better Auth migration complexity | Start simple (email/password), add OAuth post-MVP, document schema extensions |

## Next Steps

1. Proceed to Phase 1: Create `data-model.md` with database schema
2. Generate `contracts/openapi.yml` with API endpoint specifications
3. Write `quickstart.md` for developer onboarding
4. Update agent context files with technology stack
5. Run `/sp.tasks` to generate implementation task breakdown
