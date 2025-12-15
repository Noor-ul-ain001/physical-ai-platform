import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import ChatbotWidget from '../../chat/ChatbotWidget';
import UserAuthButton from '../../auth/UserAuthButton';
import TranslationButton from '../../buttons/TranslationButton';
import PersonalizationModal from '../../personalization/PersonalizationModal';
import { useTranslation } from '../../../contexts/TranslationContext';
import styles from './FloatingChatIcon.module.css';

type FloatingChatIconProps = {
  chapterId?: string;
  className?: string;
};

export default function FloatingChatIcon({
  chapterId = 'default',
  className
}: FloatingChatIconProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [activeTab, setActiveTab] = useState<'chat' | 'auth' | 'translate' | 'personalize'>('chat');
  const [showPersonalizationModal, setShowPersonalizationModal] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);
  const { isTranslating, translationError } = useTranslation();

  // Close the panel when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (containerRef.current && !containerRef.current.contains(event.target as Node)) {
        if (isOpen) {
          setIsOpen(false);
        }
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  const togglePanel = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      // Default to chat when opening
      setActiveTab('chat');
    }
  };

  const handleLanguageChange = (language: 'en' | 'ur') => {
    console.log(`Language changed to: ${language}`);
    // The translation context handles the actual language change
  };

  const handlePersonalizeClick = () => {
    setActiveTab('personalize');
    setShowPersonalizationModal(true);
    setIsOpen(true); // Ensure the panel is open when showing personalization modal
  };

  return (
    <div className={clsx(styles.floatingContainer, className)} ref={containerRef}>
      {/* Floating Icon Button */}
      <button
        className={clsx(styles.floatingButton, isOpen && styles.open)}
        onClick={togglePanel}
        aria-label={isOpen ? "Close chat panel" : "Open chat panel"}
        title="AI Assistant"
      >
        <span className={styles.icon}>🤖</span>
        {!isOpen && (
          <span className={styles.badge}>
            <span className={styles.pulse}></span>
          </span>
        )}
      </button>

      {/* Expanded Panel */}
      {isOpen && (
        <div className={styles.panel}>
          {/* Tab Navigation */}
          <div className={styles.tabNavigation}>
            <button
              className={clsx(styles.tabButton, activeTab === 'chat' && styles.active)}
              onClick={() => setActiveTab('chat')}
              aria-label="Open chat"
            >
              💬
            </button>
            <button
              className={clsx(styles.tabButton, activeTab === 'auth' && styles.active)}
              onClick={() => setActiveTab('auth')}
              aria-label="Authentication"
            >
              🔐
            </button>
            <button
              className={clsx(styles.tabButton, activeTab === 'translate' && styles.active)}
              onClick={() => setActiveTab('translate')}
              aria-label="Translation"
            >
              🌐
            </button>
            <button
              className={clsx(styles.tabButton, activeTab === 'personalize' && styles.active)}
              onClick={handlePersonalizeClick}
              aria-label="Personalization"
            >
              🎯
            </button>
          </div>

          {/* Tab Content */}
          <div className={styles.tabContent}>
            {activeTab === 'chat' && (
              <div className={styles.chatContainer}>
                <ChatbotWidget className={styles.chatWidget} />
              </div>
            )}

            {activeTab === 'auth' && (
              <div className={styles.authContainer}>
                <h3>Authentication</h3>
                <UserAuthButton />
              </div>
            )}

            {activeTab === 'translate' && (
              <div className={styles.translateContainer}>
                <h3>Translation</h3>
                <p>Translate this content to your preferred language:</p>
                <TranslationButton
                  chapterId={chapterId}
                  onLanguageChange={handleLanguageChange}
                />
                {isTranslating && (
                  <p className={styles.statusMessage}>Translating content...</p>
                )}
                {translationError && (
                  <p className={styles.errorMessage}>{translationError}</p>
                )}
                <p className={styles.helpText}>
                  Switch between English and Urdu to enhance your learning experience.
                </p>
              </div>
            )}

            {activeTab === 'personalize' && (
              <div className={styles.personalizeContainer}>
                <h3>Personalization</h3>
                <p>Customize your learning experience based on your profile:</p>
                <button
                  className={styles.personalizeButton}
                  onClick={handlePersonalizeClick}
                >
                  Adjust Learning Preferences
                </button>
                <p className={styles.helpText}>
                  Tailor content to your hardware access, experience level, and goals.
                </p>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Personalization Modal */}
      {showPersonalizationModal && (
        <PersonalizationModal
          isOpen={showPersonalizationModal}
          onClose={() => setShowPersonalizationModal(false)}
          chapterId={chapterId}
        />
      )}
    </div>
  );
}