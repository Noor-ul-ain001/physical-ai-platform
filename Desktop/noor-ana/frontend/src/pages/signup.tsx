import React, { useState } from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/auth/SignupForm';
import ProfileQuestionnaire from '../components/auth/ProfileQuestionnaire';
import styles from './signup.module.css';

type FormData = {
  email: string;
  password: string;
  confirmPassword: string;
};

type ProfileData = {
  hardwareProfile: {
    gpu: string;
    roboticsKit: string;
    experienceLevel: string;
  };
  learningGoals: string[];
};

export default function SignupPage(): JSX.Element {
  const [step, setStep] = useState<'signup' | 'profile' | 'complete'>('signup');
  const [userFormData, setUserFormData] = useState<FormData | null>(null);

  const handleSignup = (formData: FormData) => {
    // In a real implementation, this would call the backend API
    console.log('Signing up user:', formData);
    setUserFormData(formData);
    setStep('profile');
  };

  const handleProfileSubmit = (profileData: ProfileData) => {
    // In a real implementation, this would call the backend API to save profile
    console.log('Profile submitted:', profileData);
    setStep('complete');
  };

  return (
    <Layout title="Sign Up" description="Create your Physical AI & Humanoid Robotics account">
      <main className={styles.main}>
        <div className={styles.container}>
          {step === 'signup' && (
            <div className={styles.stepContainer}>
              <h1>Create Your Account</h1>
              <p className={styles.subtitle}>
                Join the Physical AI & Humanoid Robotics educational platform
              </p>
              <SignupForm onSignup={handleSignup} />
              <div className={styles.loginLink}>
                Already have an account? <a href="/signin">Sign in here</a>
              </div>
            </div>
          )}

          {step === 'profile' && (
            <div className={styles.stepContainer}>
              <h1>Welcome! Complete Your Profile</h1>
              <p className={styles.subtitle}>
                Help us personalize your learning experience
              </p>
              <ProfileQuestionnaire 
                onSubmit={handleProfileSubmit} 
                initialData={{
                  hardwareProfile: {
                    gpu: '',
                    roboticsKit: '',
                    experienceLevel: ''
                  },
                  learningGoals: []
                }} 
              />
            </div>
          )}

          {step === 'complete' && (
            <div className={styles.stepContainer}>
              <h1>Account Created Successfully!</h1>
              <p className={styles.subtitle}>
                Welcome to the Physical AI & Humanoid Robotics platform.
              </p>
              <div className={styles.successMessage}>
                <p>Your account has been created and your profile is set up.</p>
                <p>You can now access the curriculum and start learning.</p>
              </div>
              <a href="/" className={styles.continueButton}>
                Continue to Dashboard
              </a>
            </div>
          )}
        </div>
      </main>
    </Layout>
  );
}