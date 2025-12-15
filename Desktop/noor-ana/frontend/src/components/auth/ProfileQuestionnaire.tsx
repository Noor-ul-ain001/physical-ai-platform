import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './ProfileQuestionnaire.module.css';

type HardwareOption = {
  id: string;
  label: string;
  description: string;
};

type ExperienceLevel = {
  id: string;
  label: string;
  description: string;
};

type ProfileData = {
  hardwareProfile: {
    gpu: string;
    roboticsKit: string;
    experienceLevel: string;
  };
  learningGoals: string[];
};

type ProfileQuestionnaireProps = {
  initialData?: ProfileData;
  onSubmit?: (profileData: ProfileData) => void;
  className?: string;
};

export default function ProfileQuestionnaire({ 
  initialData, 
  onSubmit, 
  className 
}: ProfileQuestionnaireProps) {
  const [profileData, setProfileData] = useState<ProfileData>(
    initialData || {
      hardwareProfile: {
        gpu: '',
        roboticsKit: '',
        experienceLevel: ''
      },
      learningGoals: []
    }
  );
  const [currentStep, setCurrentStep] = useState(0);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const hardwareOptions: HardwareOption[] = [
    {
      id: 'none',
      label: 'No Hardware',
      description: 'I don\'t have access to robotics hardware'
    },
    {
      id: 'jetson',
      label: 'Jetson Orin',
      description: 'I have access to NVIDIA Jetson Orin development kit'
    },
    {
      id: 'full-robot',
      label: 'Full Robot',
      description: 'I have access to a full robot platform (e.g., Unitree Go1)'
    }
  ];

  const experienceLevels: ExperienceLevel[] = [
    {
      id: 'beginner',
      label: 'Beginner',
      description: 'New to robotics and AI'
    },
    {
      id: 'intermediate',
      label: 'Intermediate',
      description: 'Have some experience with programming/robotics'
    },
    {
      id: 'advanced',
      label: 'Advanced',
      description: 'Experienced with robotics systems'
    }
  ];

  const learningGoalsOptions = [
    'ROS 2 Development',
    'Simulation Environments',
    'NVIDIA Isaac Sim',
    'VLA Models',
    'Hardware Integration',
    'AI for Robotics'
  ];

  const stepTitles = ['Hardware Access', 'Experience Level', 'Learning Goals'];

  const handleHardwareChange = (optionId: string) => {
    setProfileData(prev => ({
      ...prev,
      hardwareProfile: {
        ...prev.hardwareProfile,
        roboticsKit: optionId
      }
    }));
  };

  const handleExperienceChange = (levelId: string) => {
    setProfileData(prev => ({
      ...prev,
      hardwareProfile: {
        ...prev.hardwareProfile,
        experienceLevel: levelId
      }
    }));
  };

  const handleGoalToggle = (goal: string) => {
    setProfileData(prev => {
      const newGoals = prev.learningGoals.includes(goal)
        ? prev.learningGoals.filter(g => g !== goal)
        : [...prev.learningGoals, goal];
      
      return {
        ...prev,
        learningGoals: newGoals
      };
    });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);
    
    try {
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000));
      onSubmit?.(profileData);
    } catch (error) {
      console.error('Error submitting profile:', error);
    } finally {
      setIsSubmitting(false);
    }
  };

  const nextStep = () => {
    if (currentStep < 2) {
      setCurrentStep(currentStep + 1);
    }
  };

  const prevStep = () => {
    if (currentStep > 0) {
      setCurrentStep(currentStep - 1);
    }
  };

  const isStepValid = (step: number): boolean => {
    switch(step) {
      case 0:
        return true; // Optional
      case 1:
        return true; // Optional
      case 2:
        return profileData.learningGoals.length > 0; // Require at least one goal
      default:
        return false;
    }
  };

  const getStepStatus = (index: number) => {
    if (index === currentStep) return 'active';
    if (index < currentStep) return 'completed';
    return 'pending';
  };

  return (
    <div className={clsx(styles.questionnaire, className)}>
      <h2>Profile Questionnaire</h2>
      <p className={styles.description}>
        Help us personalize your learning experience by answering a few questions about your background and goals.
      </p>

      {/* Progress Steps */}
      <div className={styles.stepProgress}>
        {stepTitles.map((title, index) => (
          <div 
            key={index} 
            className={clsx(
              styles.step,
              styles[getStepStatus(index)]
            )}
          >
            <div className={styles.stepIndicator}>
              {getStepStatus(index) === 'completed' ? '✓' : index + 1}
            </div>
            <span className={styles.stepLabel}>{title}</span>
          </div>
        ))}
      </div>

      <form onSubmit={handleSubmit} className={styles.form}>
        <div className={styles.stepContent}>
          {currentStep === 0 && (
            <div>
              <h3>What hardware access do you have?</h3>
              <p className={styles.stepDescription}>
                This helps us show you relevant examples and exercises.
              </p>
              
              <div className={styles.optionList}>
                {hardwareOptions.map(option => (
                  <label 
                    key={option.id} 
                    className={clsx(
                      styles.option,
                      profileData.hardwareProfile.roboticsKit === option.id && styles.optionSelected
                    )}
                  >
                    <input
                      type="radio"
                      name="hardware"
                      value={option.id}
                      checked={profileData.hardwareProfile.roboticsKit === option.id}
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
          )}

          {currentStep === 1 && (
            <div>
              <h3>What is your experience level?</h3>
              <p className={styles.stepDescription}>
                This helps us recommend appropriate content for your skill level.
              </p>
              
              <div className={styles.optionList}>
                {experienceLevels.map(level => (
                  <label 
                    key={level.id} 
                    className={clsx(
                      styles.option,
                      profileData.hardwareProfile.experienceLevel === level.id && styles.optionSelected
                    )}
                  >
                    <input
                      type="radio"
                      name="experience"
                      value={level.id}
                      checked={profileData.hardwareProfile.experienceLevel === level.id}
                      onChange={() => handleExperienceChange(level.id)}
                      className={styles.optionInput}
                    />
                    <div className={styles.optionContent}>
                      <span className={styles.optionLabel}>{level.label}</span>
                      <span className={styles.optionDescription}>{level.description}</span>
                    </div>
                  </label>
                ))}
              </div>
            </div>
          )}

          {currentStep === 2 && (
            <div>
              <h3>What are your learning goals?</h3>
              <p className={styles.stepDescription}>
                Select all that apply to help us focus on what matters most to you.
              </p>
              
              <div className={styles.checkboxList}>
                {learningGoalsOptions.map(goal => (
                  <label key={goal} className={styles.checkboxOption}>
                    <input
                      type="checkbox"
                      checked={profileData.learningGoals.includes(goal)}
                      onChange={() => handleGoalToggle(goal)}
                      className={styles.checkboxInput}
                    />
                    <span className={styles.checkboxLabel}>{goal}</span>
                  </label>
                ))}
              </div>
              
              {profileData.learningGoals.length === 0 && currentStep === 2 && (
                <div className={styles.errorMessage}>
                  Please select at least one learning goal to continue.
                </div>
              )}
            </div>
          )}
        </div>

        <div className={styles.navigation}>
          {currentStep > 0 && (
            <button 
              type="button" 
              onClick={prevStep}
              className={styles.navButton}
            >
              Previous
            </button>
          )}
          
          {currentStep < 2 && (
            <button 
              type="button" 
              onClick={nextStep}
              className={styles.navButton}
            >
              Next
            </button>
          )}
          
          {currentStep === 2 && (
            <button 
              type="submit"
              className={styles.submitButton}
              disabled={isSubmitting || !isStepValid(2)}
            >
              {isSubmitting ? (
                <>
                  <span className={styles.spinner}></span>
                  Saving Profile...
                </>
              ) : (
                'Complete Profile'
              )}
            </button>
          )}
        </div>
      </form>
    </div>
  );
}

// Add this CSS for the error message
// Add to your CSS module:
/*
.errorMessage {
  color: #ff6b6b;
  font-size: 0.875rem;
  margin-top: 1rem;
  padding: 0.5rem;
  background: rgba(255, 107, 107, 0.1);
  border-radius: 0.375rem;
  text-align: center;
}
*/