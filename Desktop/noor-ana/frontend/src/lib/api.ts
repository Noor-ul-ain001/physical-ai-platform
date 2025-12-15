/**
 * API Client for Physical AI & Humanoid Robotics Platform
 * Handles all HTTP requests to the backend with automatic authentication
 */

// Fix for Docusaurus - process is not defined in browser
// API URL can be configured via window.API_CONFIG or defaults to localhost
const getApiBaseUrl = (): string => {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000';
  }

  // Check for config in window object
  const windowConfig = (window as any).API_CONFIG;
  if (windowConfig?.baseUrl) {
    return windowConfig.baseUrl;
  }

  // Default to localhost for development
  return 'http://localhost:8000';
};

const API_BASE_URL = getApiBaseUrl();

// Token management
export const TokenManager = {
  getToken: (): string | null => {
    if (typeof window === 'undefined') return null;
    return localStorage.getItem('auth_token');
  },

  setToken: (token: string): void => {
    if (typeof window === 'undefined') return;
    localStorage.setItem('auth_token', token);
  },

  removeToken: (): void => {
    if (typeof window === 'undefined') return;
    localStorage.removeItem('auth_token');
  }
};

// Types
export interface ApiResponse<T> {
  data?: T;
  error?: string;
  status: number;
}

export interface SignupRequest {
  email: string;
  password: string;
}

export interface LoginRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  access_token: string;
  token_type: string;
}

export interface UserProfile {
  id: string;
  email: string;
  hardware_profile?: {
    gpu?: string;
    robotics_kit?: string;
    experience_level?: string;
  };
  learning_goals?: string[];
  accessibility_prefs?: Record<string, any>;
  is_active: boolean;
  created_at?: string;
}

export interface ChatRequest {
  query: string;
  session_id?: string;
  top_k?: number;
}

export interface Citation {
  source: string;
  excerpt: string;
  similarity: number;
}

export interface ChatResponse {
  answer: string;
  citations: Citation[];
  confidence: number;
  retrieval_time_ms: number;
}

export interface Bookmark {
  id: string;
  user_id: string;
  chapter_id: string;
  section_id?: string;
  notes?: string;
  created_at?: string;
}

export interface Progress {
  id: string;
  user_id: string;
  chapter_id: string;
  completion_percentage: number;
  last_accessed?: string;
  time_spent_seconds: number;
  exercises_completed?: string[];
  quiz_scores?: Record<string, number>;
}

// Base fetch function with auth
async function apiFetch<T>(
  endpoint: string,
  options: RequestInit = {}
): Promise<ApiResponse<T>> {
  const token = TokenManager.getToken();

  const headers: Record<string, string> = {
    'Content-Type': 'application/json',
    ...(options.headers as Record<string, string> || {})
  };

  // Add Authorization header if token exists and endpoint requires auth
  if (token && !endpoint.includes('/signup') && !endpoint.includes('/login') && !endpoint.includes('/chat')) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  try {
    const response = await fetch(`${API_BASE_URL}${endpoint}`, {
      ...options,
      headers
    });

    const data = await response.json().catch(() => null);

    if (!response.ok) {
      return {
        error: data?.detail || `HTTP ${response.status}: ${response.statusText}`,
        status: response.status
      };
    }

    return {
      data,
      status: response.status
    };
  } catch (error) {
    return {
      error: error instanceof Error ? error.message : 'Network error',
      status: 0
    };
  }
}

// Auth API
export const AuthAPI = {
  signup: async (request: SignupRequest): Promise<ApiResponse<UserProfile>> => {
    return apiFetch<UserProfile>('/api/users/signup', {
      method: 'POST',
      body: JSON.stringify(request)
    });
  },

  login: async (request: LoginRequest): Promise<ApiResponse<AuthResponse>> => {
    const response = await apiFetch<AuthResponse>('/api/users/login', {
      method: 'POST',
      body: JSON.stringify(request)
    });

    // Store token on successful login
    if (response.data?.access_token) {
      TokenManager.setToken(response.data.access_token);
    }

    return response;
  },

  logout: (): void => {
    TokenManager.removeToken();
  },

  getCurrentUser: async (): Promise<ApiResponse<UserProfile>> => {
    return apiFetch<UserProfile>('/api/users/me');
  },

  updateProfile: async (profile: Partial<UserProfile>): Promise<ApiResponse<UserProfile>> => {
    return apiFetch<UserProfile>('/api/users/me', {
      method: 'PATCH',
      body: JSON.stringify(profile)
    });
  }
};

// Chat API
export const ChatAPI = {
  sendMessage: async (request: ChatRequest): Promise<ApiResponse<ChatResponse>> => {
    return apiFetch<ChatResponse>('/api/chat', {
      method: 'POST',
      body: JSON.stringify(request)
    });
  },

  sendSelectiveMessage: async (
    query: string,
    selectedText: string,
    pageContext?: string
  ): Promise<ApiResponse<ChatResponse>> => {
    return apiFetch<ChatResponse>('/api/chat/selective', {
      method: 'POST',
      body: JSON.stringify({
        query,
        selected_text: selectedText,
        page_context: pageContext
      })
    });
  }
};

// Bookmark API
export const BookmarkAPI = {
  create: async (
    chapterId: string,
    sectionId?: string,
    notes?: string
  ): Promise<ApiResponse<Bookmark>> => {
    return apiFetch<Bookmark>('/api/bookmarks', {
      method: 'POST',
      body: JSON.stringify({
        chapter_id: chapterId,
        section_id: sectionId,
        notes
      })
    });
  },

  getAll: async (): Promise<ApiResponse<Bookmark[]>> => {
    return apiFetch<Bookmark[]>('/api/bookmarks');
  },

  delete: async (bookmarkId: string): Promise<ApiResponse<{message: string}>> => {
    return apiFetch(`/api/bookmarks/${bookmarkId}`, {
      method: 'DELETE'
    });
  }
};

// Progress API
export const ProgressAPI = {
  update: async (
    chapterId: string,
    completionPercentage: number,
    timeSpentSeconds: number = 0,
    exercisesCompleted: string[] = [],
    quizScores: Record<string, number> = {}
  ): Promise<ApiResponse<Progress>> => {
    return apiFetch<Progress>('/api/progress', {
      method: 'POST',
      body: JSON.stringify({
        chapter_id: chapterId,
        completion_percentage: completionPercentage,
        time_spent_seconds: timeSpentSeconds,
        exercises_completed: exercisesCompleted,
        quiz_scores: quizScores
      })
    });
  },

  getAll: async (): Promise<ApiResponse<Progress[]>> => {
    return apiFetch<Progress[]>('/api/progress');
  }
};

// Content API (Personalization & Translation)
export const ContentAPI = {
  personalize: async (
    chapterId: string,
    hardwareProfile?: any,
    learningGoals?: string[]
  ): Promise<ApiResponse<any>> => {
    return apiFetch('/api/content/personalize', {
      method: 'POST',
      body: JSON.stringify({
        chapter_id: chapterId,
        hardware_profile: hardwareProfile,
        learning_goals: learningGoals
      })
    });
  },

  translate: async (chapterId: string, targetLanguage: string = 'ur'): Promise<ApiResponse<any>> => {
    return apiFetch('/api/translate', {
      method: 'POST',
      body: JSON.stringify({
        chapter_id: chapterId,
        target_language: targetLanguage
      })
    });
  }
};

export default {
  Auth: AuthAPI,
  Chat: ChatAPI,
  Bookmark: BookmarkAPI,
  Progress: ProgressAPI,
  Content: ContentAPI
};