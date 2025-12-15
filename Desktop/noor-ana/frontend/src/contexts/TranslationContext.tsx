import React, { createContext, useContext, useReducer, useEffect } from 'react';

// Define types
type TranslationState = {
  isTranslated: boolean;
  isTranslating: boolean;
  translatedContent: string | null;
  currentLanguage: string;
  translationError: string | null;
};

type TranslationAction =
  | { type: 'START_TRANSLATION' }
  | { type: 'TRANSLATION_SUCCESS'; payload: string }
  | { type: 'TRANSLATION_ERROR'; payload: string }
  | { type: 'RESET_TRANSLATION' }
  | { type: 'SET_LANGUAGE'; payload: string };

// Initial state
const initialState: TranslationState = {
  isTranslated: false,
  isTranslating: false,
  translatedContent: null,
  currentLanguage: 'en',
  translationError: null
};

// Reducer function
const translationReducer = (state: TranslationState, action: TranslationAction): TranslationState => {
  switch (action.type) {
    case 'START_TRANSLATION':
      return {
        ...state,
        isTranslating: true,
        translationError: null
      };
    
    case 'TRANSLATION_SUCCESS':
      return {
        ...state,
        isTranslated: true,
        isTranslating: false,
        translatedContent: action.payload,
        currentLanguage: 'ur'
      };
    
    case 'TRANSLATION_ERROR':
      return {
        ...state,
        isTranslating: false,
        translationError: action.payload
      };
    
    case 'RESET_TRANSLATION':
      return {
        ...initialState,
        currentLanguage: 'en'
      };
    
    case 'SET_LANGUAGE':
      return {
        ...state,
        currentLanguage: action.payload
      };
    
    default:
      return state;
  }
};

// Create context
interface TranslationContextType extends TranslationState {
  translateContent: (chapterId: string) => Promise<void>;
  resetTranslation: () => void;
  setLanguage: (lang: string) => void;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

// Provider component
type TranslationProviderProps = {
  children: React.ReactNode;
};

export const TranslationProvider: React.FC<TranslationProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(translationReducer, initialState);

  const translateContent = async (chapterId: string) => {
    dispatch({ type: 'START_TRANSLATION' });
    
    try {
      // In a real implementation, this would call the backend API
      // const response = await fetch(`/api/translate`, {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify({ chapterId, targetLanguage: 'ur' })
      // });
      
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1500));
      
      // Simulate response
      const mockTranslatedContent = `[URDU TRANSLATION OF CHAPTER ${chapterId}] 
      یہ ایک مشین ترجمہ ہے جو اصل مواد کو اردو میں تبدیل کرتا ہے۔ 
      تمام کوڈ بلاک اور لنکس محفوظ رکھے جاتے ہیں۔`;
      
      dispatch({ 
        type: 'TRANSLATION_SUCCESS', 
        payload: mockTranslatedContent 
      });
    } catch (error) {
      dispatch({ 
        type: 'TRANSLATION_ERROR', 
        payload: 'Translation failed. Please try again.' 
      });
    }
  };

  const resetTranslation = () => {
    dispatch({ type: 'RESET_TRANSLATION' });
  };

  const setLanguage = (lang: string) => {
    dispatch({ type: 'SET_LANGUAGE', payload: lang });
  };

  const value = {
    ...state,
    translateContent,
    resetTranslation,
    setLanguage
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

// Custom hook to use the translation context
export const useTranslation = (): TranslationContextType => {
  const context = useContext(TranslationContext);
  
  if (context === undefined) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }
  
  return context;
};