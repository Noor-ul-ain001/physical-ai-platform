# Gemini API Setup Guide

This project now uses **Google's Free Gemini API** instead of OpenAI!

## 🆓 Get Your Free Gemini API Key

1. **Go to Google AI Studio**: https://makersuite.google.com/app/apikey

2. **Sign in** with your Google account

3. **Click "Create API Key"**

4. **Copy your API key** - it looks like: `AIzaSy...`

5. **Add it to your `.env` file**:
   ```bash
   GEMINI_API_KEY=AIzaSy...your-key-here
   ```

## ✅ What's Changed

### Free Tier Benefits
- ✅ **Gemini 1.5 Flash** - Fast, free chat model
- ✅ **text-embedding-004** - Free embeddings (768 dimensions)
- ✅ **60 requests per minute** - Generous free quota
- ✅ **No credit card required**

### Technical Changes
- **Embeddings**: 768 dimensions (was 1536 with OpenAI)
- **Chat Model**: gemini-1.5-flash (was gpt-4-turbo)
- **Cost**: $0.00 🎉 (was ~$0.03 per 1K tokens)

## 📦 Installation

### Backend Setup

```bash
cd backend

# Install new dependencies
pip install google-generativeai==0.3.2

# Or install all requirements
pip install -r requirements.txt

# Update your .env file
echo "GEMINI_API_KEY=your-api-key-here" >> .env

# Start the server
uvicorn src.main:app --reload
```

### Frontend Setup

```bash
cd frontend

# Start development server
npm start
```

The frontend will run on http://localhost:3000 and connect to backend at http://localhost:8000

## 🔧 Configuration

### Backend `.env` File

```bash
# Required
DATABASE_URL=postgresql://user:pass@neon-host/db
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
GEMINI_API_KEY=AIzaSy...your-gemini-key
REDIS_URL=redis://default:pass@upstash-url
BETTER_AUTH_SECRET=your-32-char-secret
BETTER_AUTH_URL=http://localhost:8000

# Optional
GOOGLE_TRANSLATE_API_KEY=your-translate-key
```

### Frontend Configuration

No environment variables needed for development! The frontend automatically connects to `http://localhost:8000`.

For production, set `window.API_CONFIG`:
```html
<script>
  window.API_CONFIG = {
    baseUrl: 'https://your-production-api.com'
  };
</script>
```

## 🧪 Testing the Integration

### 1. Test Embeddings

```bash
cd backend
python -c "
import asyncio
from src.core.embeddings import get_single_embedding

async def test():
    embedding = await get_single_embedding('Hello Gemini!')
    print(f'Embedding length: {len(embedding)}')
    print(f'First 5 values: {embedding[:5]}')

asyncio.run(test())
"
```

Expected output: `Embedding length: 768`

### 2. Test Chat

```bash
# Start backend
uvicorn src.main:app --reload

# In another terminal, test the API
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "top_k": 5}'
```

### 3. Test Frontend

1. Open http://localhost:3000
2. Navigate to any module chapter
3. Click chatbot icon
4. Ask: "What is ROS 2?"
5. Verify you get a response with citations

## ⚠️ Important Notes

### Qdrant Collection Reset

If you had existing collections with 1536 dimensions (OpenAI), you need to delete and recreate them:

```bash
# Option 1: Delete via Qdrant Cloud Dashboard
# Go to https://cloud.qdrant.io → Collections → Delete book_content_en and book_content_ur

# Option 2: Delete via API
curl -X DELETE "https://your-cluster.qdrant.io:6333/collections/book_content_en" \
  -H "api-key: your-qdrant-key"

curl -X DELETE "https://your-cluster.qdrant.io:6333/collections/book_content_ur" \
  -H "api-key: your-qdrant-key"

# Restart backend - collections will be recreated with 768 dimensions
uvicorn src.main:app --reload
```

### Rate Limits

Gemini Free Tier limits:
- **60 requests per minute**
- **1,500 requests per day**

If you hit limits, responses will fail. For production, consider:
- Implementing request queuing
- Using Gemini Pro (paid tier)
- Caching responses aggressively

## 📊 Performance Comparison

| Feature | OpenAI (Old) | Gemini (New) |
|---------|--------------|--------------|
| **Cost** | $0.03/1K tokens | FREE ✅ |
| **Embedding Dims** | 1536 | 768 |
| **Chat Model** | GPT-4 Turbo | Gemini 1.5 Flash |
| **Response Time** | ~1-2s | ~1-2s |
| **Quality** | Excellent | Very Good |
| **Rate Limit** | Pay per use | 60 req/min |

## 🐛 Troubleshooting

### Error: "process is not defined"

**Fixed!** This was caused by trying to use `process.env` in browser. Now using `window.API_CONFIG` instead.

### Error: "Invalid API key"

1. Check your `.env` file has correct `GEMINI_API_KEY`
2. Verify the key at https://makersuite.google.com/app/apikey
3. Make sure there are no extra spaces or quotes

### Error: "Dimension mismatch in Qdrant"

Your Qdrant collections were created for OpenAI (1536 dims). Delete and recreate them:

```bash
# See "Qdrant Collection Reset" section above
```

### Error: "Module 'google.generativeai' not found"

```bash
pip install google-generativeai==0.3.2
```

## 🚀 Next Steps

1. ✅ Get free Gemini API key
2. ✅ Update `.env` with `GEMINI_API_KEY`
3. ✅ Install dependencies: `pip install -r requirements.txt`
4. ✅ Delete old Qdrant collections (if any)
5. ✅ Start backend: `uvicorn src.main:app --reload`
6. ✅ Start frontend: `npm start`
7. ✅ Test chatbot at http://localhost:3000

## 📚 Resources

- [Gemini API Docs](https://ai.google.dev/docs)
- [Get API Key](https://makersuite.google.com/app/apikey)
- [Pricing (FREE!)](https://ai.google.dev/pricing)
- [Gemini Models](https://ai.google.dev/models/gemini)

---

**Questions?** Check the logs:
```bash
# Backend logs
tail -f backend/logs/app.log

# Frontend logs
# Check browser console (F12)
```

Enjoy your **FREE** AI-powered educational platform! 🎉
