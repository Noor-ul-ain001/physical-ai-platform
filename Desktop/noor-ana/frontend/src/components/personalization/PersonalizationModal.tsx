import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './PersonalizationModal.module.css';

type PersonalizationModalProps = {
  isOpen: boolean;
  onClose: () => void;
  chapterId: string;
  className?: string;
};

type ProfileOption = {
  id: string;
  label: string;
  description: string;
};

export default function PersonalizationModal({ 
  isOpen, 
  onClose, 
  chapterId,
  className 
}: PersonalizationModalProps): JSX.Element {
  const [profile, setProfile] = useState({
    hardwareAccess: 'none',
    experienceLevel: 'beginner',
    learningGoals: [] as string[]
  });
  
  const [isSaving, setIsSaving] = useState(false);
  
  const hardwareOptions: ProfileOption[] = [
    { id: 'none', label: 'No Hardware', description: 'I don\'t have access to robotics hardware' },
    { id: 'jetson', label: 'Jetson Orin', description: 'I have access to NVIDIA Jetson Orin development kit' },
    { id: 'full-robot', label: 'Full Robot', description: 'I have access to a full robot platform' }
  ];

  const experienceOptions: ProfileOption[] = [
    { id: 'beginner', label: 'Beginner', description: 'New to robotics and AI' },
    { id: 'intermediate', label: 'Intermediate', description: 'Have some experience with programming/robotics' },
    { id: 'advanced', label: 'Advanced', description: 'Experienced with robotics systems' }
  ];

  const learningGoalsOptions = [
    'ROS 2 Development',
    'Simulation Environments',
    'NVIDIA Isaac Sim',
    'VLA Models',
    'Hardware Integration',
    'AI for Robotics'
  ];

  // Load saved profile if exists
  useEffect(() => {
    if (isOpen) {
      const savedProfile = localStorage.getItem('personalizationProfile');
      if (savedProfile) {
        try {
          setProfile(JSON.parse(savedProfile));
        } catch (e) {
          console.error('Error loading saved profile:', e);
        }
      }
    }
  }, [isOpen]);

  const handleHardwareChange = (id: string) => {
    setProfile(prev => ({ ...prev, hardwareAccess: id }));
  };

  const handleExperienceChange = (id: string) => {
    setProfile(prev => ({ ...prev, experienceLevel: id }));
  };

  const handleGoalToggle = (goal: string) => {
    setProfile(prev => {
      const newGoals = prev.learningGoals.includes(goal)
        ? prev.learningGoals.filter(g => g !== goal)
        : [...prev.learningGoals, goal];
      
      return { ...prev, learningGoals: newGoals };
    });
  };

  const handleSave = async () => {
    setIsSaving(true);
    
    try {
      // In a real implementation, this would call the backend API
      // to apply personalization to the current chapter
      console.log('Saving personalization for chapter:', chapterId, profile);
      
      // Save profile to local storage
      localStorage.setItem('personalizationProfile', JSON.stringify(profile));
      
      // In a real app, this would trigger content personalization
      // by calling the personalization API and updating the content
      setTimeout(() => {
        onClose();
        setIsSaving(false);
      }, 500); // Simulate API call
    } catch (error) {
      console.error('Error saving personalization:', error);
      setIsSaving(false);
    }
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className={clsx(styles.modalOverlay, className)}>
      <div className={styles.modal}>
        <div className={styles.modalHeader}>
          <h2>Personalize This Chapter</h2>
          <button className={styles.closeButton} onClick={onClose} aria-label="Close">
            &times;
          </button>
        </div>
        
        <div className={styles.modalBody}>
          <div className={styles.section}>
            <h3>Hardware Access</h3>
            <p className={styles.sectionDescription}>
              Select the hardware you have access to for personalized content.
            </p>
            <div className={styles.optionList}>
              {hardwareOptions.map(option => (
                <label 
                  key={option.id}
                  className={clsx(
                    styles.option,
                    profile.hardwareAccess === option.id && styles.optionSelected
                  )}
                >
                  <input
                    type="radio"
                    name="hardwareAccess"
                    value={option.id}
                    checked={profile.hardwareAccess === option.id}
                    onChange={() => handleHardwareChange(option.id)}
                    className={styles.optionInput}
                  />
                  <div className={styles.optionContent}>
                    <span className={styles.optionLabel}>{option.label}</span>
                    <span className={styles.optionDescription}>{option.description}</span>
                  </div>
                </label>
              ))}
            </div>
          </div>
          
          <div className={styles.section}>
            <h3>Experience Level</h3>
            <p className={styles.sectionDescription}>
              Select your experience level for appropriate content complexity.
            </p>
            <div className={styles.optionList}>
              {experienceOptions.map(option => (
                <label 
                  key={option.id}
                  className={clsx(
                    styles.option,
                    profile.experienceLevel === option.id && styles.optionSelected
                  )}
                >
                  <input
                    type="radio"
                    name="experienceLevel"
                    value={option.id}
                    checked={profile.experienceLevel === option.id}
                    onChange={() => handleExperienceChange(option.id)}
                    className={styles.optionInput}
                  />
                  <div className={styles.optionContent}>
                    <span className={styles.optionLabel}>{option.label}</span>
                    <span className={styles.optionDescription}>{option.description}</span>
                  </div>
                </label>
              ))}
            </div>
          </div>
          
          <div className={styles.section}>
            <h3>Learning Goals</h3>
            <p className={styles.sectionDescription}>
              Select your learning goals to focus on relevant content.
            </p>
            <div className={styles.checkboxList}>
              {learningGoalsOptions.map(goal => (
                <label 
                  key={goal}
                  className={clsx(
                    styles.checkboxOption,
                    profile.learningGoals.includes(goal) && styles.checkboxSelected
                  )}
                >
                  <input
                    type="checkbox"
                    checked={profile.learningGoals.includes(goal)}
                    onChange={() => handleGoalToggle(goal)}
                    className={styles.checkboxInput}
                  />
                  <span className={styles.checkboxLabel}>{goal}</span>
                </label>
              ))}
            </div>
          </div>
        </div>
        
        <div className={styles.modalFooter}>
          <button 
            className={styles.cancelButton} 
            onClick={onClose}
          >
            Cancel
          </button>
          <button 
            className={styles.saveButton} 
            onClick={handleSave}
            disabled={isSaving}
          >
            {isSaving ? 'Applying...' : 'Apply Personalization'}
          </button>
        </div>
      </div>
    </div>
  );
}