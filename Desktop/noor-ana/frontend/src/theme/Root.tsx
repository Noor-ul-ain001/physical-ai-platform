import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import FloatingChatIcon from '../components/common/FloatingChatIcon/FloatingChatIcon';

// Docusaurus Root wrapper - wraps the entire app
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      <TranslationProvider>
        <>
          {children}
          <FloatingChatIcon />
        </>
      </TranslationProvider>
    </AuthProvider>
  );
}
