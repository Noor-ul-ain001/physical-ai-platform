import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { AuthAPI, UserProfile, TokenManager } from '../lib/api';

interface AuthContextType {
  user: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<{ success: boolean; error?: string }>;
  signup: (email: string, password: string) => Promise<{ success: boolean; error?: string }>;
  logout: () => void;
  refreshUser: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Check if user is already logged in on mount
  useEffect(() => {
    const initAuth = async () => {
      const token = TokenManager.getToken();
      if (token) {
        await refreshUser();
      } else {
        setIsLoading(false);
      }
    };

    initAuth();
  }, []);

  const refreshUser = async () => {
    setIsLoading(true);
    try {
      const response = await AuthAPI.getCurrentUser();
      if (response.data) {
        setUser(response.data);
      } else {
        // Token is invalid, clear it
        TokenManager.removeToken();
        setUser(null);
      }
    } catch (error) {
      TokenManager.removeToken();
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  };

  const login = async (
    email: string,
    password: string
  ): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await AuthAPI.login({ email, password });

      if (response.error) {
        return { success: false, error: response.error };
      }

      // Fetch user profile after successful login
      await refreshUser();
      return { success: true };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Login failed'
      };
    }
  };

  const signup = async (
    email: string,
    password: string
  ): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await AuthAPI.signup({ email, password });

      if (response.error) {
        return { success: false, error: response.error };
      }

      // After signup, automatically log in
      return await login(email, password);
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Signup failed'
      };
    }
  };

  const logout = () => {
    AuthAPI.logout();
    setUser(null);
  };

  const value: AuthContextType = {
    user,
    isAuthenticated: !!user,
    isLoading,
    login,
    signup,
    logout,
    refreshUser
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
