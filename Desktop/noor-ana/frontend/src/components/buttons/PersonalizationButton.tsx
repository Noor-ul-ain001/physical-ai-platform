import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './PersonalizationButton.module.css';
import { ContentAPI } from '../../lib/api';
import { useAuth } from '../../contexts/AuthContext';

type PersonalizationButtonProps = {
  chapterId: string;
  className?: string;
};

// Define types for personalization rules
type PersonalizationRules = {
  showSections: string[];
  hideSections: string[];
  codeComplexity: 'simple' | 'standard' | 'advanced';
  hardwareInstructions: 'cloud' | 'jetson' | 'full_robot';
};

export default function PersonalizationButton({
  chapterId,
  className
}: PersonalizationButtonProps): JSX.Element {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const [userProfileComplete, setUserProfileComplete] = useState(false);

  // Check if user profile is complete
  useEffect(() => {
    // In a real app, this would check user profile status from context or API
    const profileComplete = localStorage.getItem('profileComplete') === 'true';
    setUserProfileComplete(profileComplete);
  }, []);

  // Check for cached personalization on component mount
  useEffect(() => {
    const cachedRules = getCachedRules(chapterId);
    if (cachedRules) {
      setIsPersonalized(true);
    }
  }, [chapterId]);

  const getCachedRules = (chapterId: string): PersonalizationRules | null => {
    const cacheKey = `personalization_rules_${chapterId}`;
    const cachedData = localStorage.getItem(cacheKey);

    if (!cachedData) return null;

    try {
      const parsed = JSON.parse(cachedData);
      // Check if cache is still valid (24 hours TTL as per spec)
      const cacheTime = parsed.timestamp;
      const now = Date.now();
      const ttl = 24 * 60 * 60 * 1000; // 24 hours in milliseconds

      if (now - cacheTime < ttl) {
        return parsed.rules;
      } else {
        // Cache expired, remove it
        localStorage.removeItem(cacheKey);
        return null;
      }
    } catch (e) {
      console.error('Error parsing cached personalization rules:', e);
      return null;
    }
  };

  const setCachedRules = (chapterId: string, rules: PersonalizationRules) => {
    const cacheKey = `personalization_rules_${chapterId}`;
    const cacheData = {
      rules,
      timestamp: Date.now()
    };

    try {
      localStorage.setItem(cacheKey, JSON.stringify(cacheData));
    } catch (e) {
      console.error('Error caching personalization rules:', e);
    }
  };

  const handlePersonalize = async () => {
    setIsProcessing(true);

    try {
      // Check if we have cached rules for this chapter
      const cachedRules = getCachedRules(chapterId);
      if (cachedRules) {
        setIsPersonalized(true);
        console.log(`Using cached personalization rules for chapter: ${chapterId}`);
        return;
      }

      // Get user's hardware profile and learning goals from localStorage
      const hardwareProfile = JSON.parse(localStorage.getItem('hardwareProfile') || 'null');
      const learningGoals = JSON.parse(localStorage.getItem('learningGoals') || '[]');

      // Call the real backend API
      console.log(`Fetching personalization rules for chapter: ${chapterId}`);
      const response = await ContentAPI.personalize(chapterId, hardwareProfile, learningGoals);

      if (response.error) {
        throw new Error(response.error);
      }

      if (response.data) {
        // Convert API response to PersonalizationRules format
        const rules: PersonalizationRules = {
          showSections: response.data.show_sections || [],
          hideSections: response.data.hide_sections || [],
          codeComplexity: response.data.code_complexity || "standard",
          hardwareInstructions: response.data.hardware_instructions || "cloud"
        };

        // Cache the rules
        setCachedRules(chapterId, rules);
        setIsPersonalized(true);

        // In a real app, this would trigger content personalization
        // by sending the rules to a context provider or similar
      }
    } catch (error) {
      console.error('Error applying personalization:', error);
    } finally {
      setIsProcessing(false);
    }
  };

  const handleReset = () => {
    // Remove cached rules for this chapter
    localStorage.removeItem(`personalization_rules_${chapterId}`);
    setIsPersonalized(false);
    // In a real app, this would reset content to original state
  };

  if (!userProfileComplete) {
    return (
      <div className={clsx(styles.buttonContainer, className)}>
        <button
          className={clsx(styles.button, styles.inactive)}
          title="Complete your profile to enable personalization"
          disabled
        >
          <span className={styles.icon}>🎯</span>
          <span>Personalize</span>
        </button>
        <div className={styles.tooltip}>
          Complete your profile to enable personalization
        </div>
      </div>
    );
  }

  return (
    <div className={clsx(styles.buttonContainer, className)}>
      {!isPersonalized ? (
        <button
          className={clsx(styles.button, styles.personalize)}
          onClick={handlePersonalize}
          disabled={isProcessing}
        >
          <span className={styles.icon}>🎯</span>
          <span>{isProcessing ? 'Personalizing...' : 'Personalize'}</span>
        </button>
      ) : (
        <button
          className={clsx(styles.button, styles.reset)}
          onClick={handleReset}
        >
          <span className={styles.icon}>↺</span>
          <span>Reset</span>
        </button>
      )}
    </div>
  );
}