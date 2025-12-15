import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './TranslationButton.module.css';

type TranslationButtonProps = {
  chapterId?: string;
  onLanguageChange?: (language: 'en' | 'ur') => void;
  className?: string;
};

export default function TranslationButton({
  chapterId,
  onLanguageChange,
  className
}: TranslationButtonProps): JSX.Element {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');

  // Load language preference from localStorage on mount
  useEffect(() => {
    const savedLanguage = localStorage.getItem('preferred_language');
    if (savedLanguage === 'ur' || savedLanguage === 'en') {
      setLanguage(savedLanguage);
    }
  }, []);

  const toggleLanguage = () => {
    const newLanguage = language === 'en' ? 'ur' : 'en';
    setLanguage(newLanguage);

    // Save preference to localStorage
    localStorage.setItem('preferred_language', newLanguage);

    // Notify parent component of language change
    onLanguageChange?.(newLanguage);

    console.log(`Language preference changed to: ${newLanguage}`);
  };

  return (
    <div className={clsx(styles.buttonContainer, className)}>
      <button
        className={clsx(styles.button, language === 'ur' ? styles.urdu : styles.english)}
        onClick={toggleLanguage}
        title={language === 'en' ? 'Switch to Urdu' : 'Switch to English'}
      >
        <span className={styles.icon}>🌐</span>
        <span>{language === 'en' ? 'Learn in Urdu' : 'Learn in English'}</span>
      </button>
    </div>
  );
}