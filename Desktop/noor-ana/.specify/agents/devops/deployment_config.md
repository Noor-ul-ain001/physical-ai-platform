# Deployment Configuration Templates

This document provides templates for deployment configurations for the Physical AI & Humanoid Robotics educational platform.

## Frontend Deployment Configuration (Vercel)

### vercel.json
```json
{
  "version": 2,
  "name": "physical-ai-platform-frontend",
  "framework": "docusaurus",
  "settings": {
    "installCommand": "npm install",
    "buildCommand": "npm run build",
    "outputDirectory": "build",
    "devCommand": "npm run start"
  },
  "env": {
    "NEXT_PUBLIC_API_URL": "$NEXT_PUBLIC_API_URL",
    "NEXT_PUBLIC_BETTER_AUTH_URL": "$NEXT_PUBLIC_BETTER_AUTH_URL"
  },
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ],
  "headers": [
    {
      "source": "/(.*)",
      "headers": [
        {
          "key": "X-Content-Type-Options",
          "value": "nosniff"
        },
        {
          "key": "X-Frame-Options",
          "value": "DENY"
        },
        {
          "key": "X-XSS-Protection",
          "value": "1; mode=block"
        },
        {
          "key": "Strict-Transport-Security",
          "value": "max-age=63072000; includeSubDomains; preload"
        }
      ]
    }
  ]
}
```

### Environment Variables (.env.example)
```
NEXT_PUBLIC_API_URL=https://api.physical-ai-platform.com
NEXT_PUBLIC_BETTER_AUTH_URL=https://auth.physical-ai-platform.com
```

## Backend Deployment Configuration (Railway)

### Dockerfile
```
FROM python:3.11-slim

WORKDIR /app

COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/ .

EXPOSE 8000

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### railway.json
```json
{
  "build": {
    "builder": "NIXPACKS",
    "nixpacksPlan": {
      "phases": {
        "setup": {
          "nixPkgs": ["python311", "pip"]
        },
        "install": {
          "cmds": ["pip install -r requirements.txt"]
        },
        "start": {
          "cmd": "uvicorn src.main:app --host 0.0.0.0 --port $PORT"
        }
      },
      "variables": {
        "PYTHONPATH": "."
      }
    }
  },
  "deploy": {
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 3
  }
}
```

### Environment Variables
```
DATABASE_URL=postgresql://user:pass@neon-host/db
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key
OPENAI_API_KEY=sk-...
REDIS_URL=redis://default:pass@upstash-url
GOOGLE_TRANSLATE_API_KEY=your-key
BETTER_AUTH_SECRET=generate-random-secret
BETTER_AUTH_URL=https://your-auth-domain.com
ALLOWED_ORIGINS=["https://your-frontend-domain.com"]
```

## GitHub Actions Workflows

### Frontend Deployment (.github/workflows/frontend-deploy.yml)
```yaml
name: Deploy Frontend

on:
  push:
    branches: [main]
    paths: ['frontend/**', 'package.json', 'package-lock.json']

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'

      - name: Install dependencies
        run: cd frontend && npm ci

      - name: Run linting
        run: cd frontend && npm run lint

      - name: Run tests
        run: cd frontend && npm run test

      - name: Build frontend
        run: cd frontend && npm run build
        env:
          NEXT_PUBLIC_API_URL: ${{ secrets.NEXT_PUBLIC_API_URL }}
          NEXT_PUBLIC_BETTER_AUTH_URL: ${{ secrets.NEXT_PUBLIC_BETTER_AUTH_URL }}

      - name: Deploy to Vercel
        uses: amondnet/vercel-action@v25
        with:
          vercel-token: ${{ secrets.VERCEL_TOKEN }}
          vercel-org-id: ${{ secrets.VERCEL_ORG_ID }}
          vercel-project-id: ${{ secrets.VERCEL_PROJECT_ID }}
          working-directory: frontend
```

### Backend Deployment (.github/workflows/backend-deploy.yml)
```yaml
name: Deploy Backend

on:
  push:
    branches: [main]
    paths: ['backend/**', 'requirements.txt']

jobs:
  deploy:
    runs-on: ubuntu-latest
    services:
      postgres:
        image: postgres:13
        env:
          POSTGRES_PASSWORD: postgres
          POSTGRES_DB: test_db
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
      redis:
        image: redis:alpine
        options: >-
          --health-cmd "redis-cli ping"
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Setup Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          cd backend
          pip install -r requirements.txt
          pip install pytest pytest-cov

      - name: Run linting
        run: |
          cd backend
          # Add linting commands here (e.g., flake8, ruff)

      - name: Run tests
        run: |
          cd backend
          pytest tests/unit/ -v
          pytest tests/integration/test_hallucination.py -v

      - name: Deploy to Railway
        uses: actions-hub/railway-deploy@main
        with:
          railway_token: ${{ secrets.RAILWAY_TOKEN }}
          environment_id: ${{ secrets.RAILWAY_ENV_ID }}
```

### CI Tests Workflow (.github/workflows/tests.yml)
```yaml
name: Run Tests

on:
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    services:
      postgres:
        image: postgres:13
        env:
          POSTGRES_PASSWORD: postgres
          POSTGRES_DB: test_db
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
      redis:
        image: redis:alpine
        options: >-
          --health-cmd "redis-cli ping"
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5

    strategy:
      matrix:
        python-version: [3.11]

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Setup Python ${{ matrix.python-version }}
        uses: actions/setup-python@v4
        with:
          python-version: ${{ matrix.python-version }}

      - name: Install dependencies
        run: |
          cd backend
          pip install -r requirements.txt
          pip install pytest pytest-cov

      - name: Run frontend tests
        run: |
          cd frontend
          npm ci
          npm run test

      - name: Run backend unit tests
        run: |
          cd backend
          pytest tests/unit/ -v

      - name: Run hallucination tests
        run: |
          cd backend
          pytest tests/integration/test_hallucination.py -v

      - name: Run E2E tests
        run: |
          cd frontend
          # Add Playwright or other E2E tests here
```

## Infrastructure as Code (Terraform)

### main.tf
```hcl
# Define providers and backend configuration
terraform {
  required_providers {
    vercel = {
      source  = "vercel/vercel"
      version = "~> 0.4.0"
    }
  }
  backend "remote" {
    # Configure remote backend for state management
  }
}

# Vercel Project Configuration
resource "vercel_project" "frontend" {
  name        = "physical-ai-platform-frontend"
  framework   = "docusaurus"
  root_directory = "frontend"

  environment = [
    {
      key    = "NEXT_PUBLIC_API_URL"
      value  = var.api_url
      target = ["production", "preview"]
    },
    {
      key    = "NEXT_PUBLIC_BETTER_AUTH_URL"
      value  = var.auth_url
      target = ["production", "preview"]
    }
  ]

  build_command = "npm run build"
  output_directory = "build"
  install_command = "npm install"
}

# Variables
variable "api_url" {
  description = "Backend API URL"
  type        = string
}

variable "auth_url" {
  description = "Better Auth URL"
  type        = string
}
```

## Monitoring Configuration

### Sentry Configuration (sentry.client.config.js)
```javascript
import * as Sentry from '@sentry/react';

Sentry.init({
  dsn: process.env.REACT_APP_SENTRY_DSN,
  integrations: [
    new Sentry.BrowserTracing(),
    new Sentry.Replay(),
  ],
  tracesSampleRate: 1.0,
  replaysSessionSampleRate: 0.1,
  replaysOnErrorSampleRate: 1.0,
});
```

### Sentry Configuration (sentry.server.config.js)
```javascript
import * as Sentry from '@sentry/node';

Sentry.init({
  dsn: process.env.SENTRY_DSN,
  tracesSampleRate: 1.0,
});
```