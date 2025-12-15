// Enhanced SignupForm component with password strength
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './SignupForm.module.css';
import { useAuth } from '../../contexts/AuthContext';

type FormData = {
  email: string;
  password: string;
  confirmPassword: string;
};

type PasswordStrength = {
  score: number;
  text: string;
  color: string;
};

export default function SignupForm({ onSignup, className }) {
  const { signup } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    confirmPassword: ''
  });
  const [errors, setErrors] = useState<Partial<FormData>>({});
  const [isLoading, setIsLoading] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState<PasswordStrength>({
    score: 0,
    text: '',
    color: '#ff6b6b'
  });
  const [showSuccess, setShowSuccess] = useState(false);

  const checkPasswordStrength = (password: string): PasswordStrength => {
    let score = 0;
    let text = '';
    let color = '#ff6b6b';
    
    if (!password) {
      return { score: 0, text: '', color };
    }
    
    // Length check
    if (password.length >= 8) score += 1;
    if (password.length >= 12) score += 1;
    
    // Complexity checks
    if (/[A-Z]/.test(password)) score += 1;
    if (/[a-z]/.test(password)) score += 1;
    if (/[0-9]/.test(password)) score += 1;
    if (/[^A-Za-z0-9]/.test(password)) score += 1;
    
    // Determine strength level
    if (score <= 2) {
      text = 'Weak';
      color = '#ff6b6b';
    } else if (score <= 4) {
      text = 'Medium';
      color = '#ffd93d';
    } else {
      text = 'Strong';
      color = '#4FA675';
    }
    
    return { score, text, color };
  };

  useEffect(() => {
    setPasswordStrength(checkPasswordStrength(formData.password));
  }, [formData.password]);

  const validate = (): boolean => {
    const newErrors: Partial<FormData> = {};
    
    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
      newErrors.email = 'Email address is invalid';
    }
    
    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    } else if (passwordStrength.score <= 2) {
      newErrors.password = 'Please use a stronger password';
    }
    
    if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }
    
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
    
    // Clear error when user starts typing
    if (errors[name as keyof FormData]) {
      setErrors(prev => ({
        ...prev,
        [name]: undefined
      }));
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validate()) {
      return;
    }

    setIsLoading(true);

    try {
      const result = await signup(formData.email, formData.password);

      if (!result.success) {
        setErrors({ email: result.error || 'Signup failed. Please try again.' });
      } else {
        onSignup?.(formData);
        setShowSuccess(true);
        
        // Redirect after showing success message
        setTimeout(() => {
          window.location.href = '/dashboard';
        }, 2000);
      }
    } catch (error) {
      console.error('Signup error:', error);
      setErrors({ email: 'An unexpected error occurred. Please try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  if (showSuccess) {
    return (
      <div className={clsx(styles.signupForm, styles.success)}>
        <div className={styles.successIcon}>✓</div>
        <h3>Account Created Successfully!</h3>
        <p>Redirecting to your dashboard...</p>
      </div>
    );
  }

  return (
    <div className={clsx(styles.signupForm, className)}>
      <h2>Create Account</h2>
      <form onSubmit={handleSubmit} className={styles.form}>
        <div className={styles.inputGroup}>
          <label htmlFor="email" className={styles.label}>Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            className={clsx(styles.input, errors.email && styles.inputError)}
            placeholder="your.email@example.com"
          />
          {errors.email && <span className={styles.error}>{errors.email}</span>}
        </div>
        
        <div className={styles.inputGroup}>
          <label htmlFor="password" className={styles.label}>Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            className={clsx(styles.input, errors.password && styles.inputError)}
            placeholder="At least 8 characters"
          />
          {formData.password && (
            <div className={styles.passwordStrength}>
              <div className={styles.strengthBar}>
                <div 
                  className={styles.strengthFill}
                  style={{
                    width: `${(passwordStrength.score / 6) * 100}%`,
                    background: passwordStrength.color
                  }}
                />
              </div>
              <div className={styles.strengthText}>
                Password strength: <strong style={{ color: passwordStrength.color }}>
                  {passwordStrength.text}
                </strong>
              </div>
            </div>
          )}
          {errors.password && <span className={styles.error}>{errors.password}</span>}
        </div>
        
        <div className={styles.inputGroup}>
          <label htmlFor="confirmPassword" className={styles.label}>Confirm Password</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            className={clsx(styles.input, errors.confirmPassword && styles.inputError)}
            placeholder="Confirm your password"
          />
          {errors.confirmPassword && <span className={styles.error}>{errors.confirmPassword}</span>}
        </div>
        
        <div className={styles.terms}>
          By signing up, you agree to our{' '}
          <a href="/terms">Terms of Service</a> and{' '}
          <a href="/privacy">Privacy Policy</a>.
        </div>
        
        <button 
          type="submit" 
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? (
            <>
              <span className={styles.spinner}></span>
              Creating Account...
            </>
          ) : (
            'Sign Up'
          )}
        </button>
      </form>
    </div>
  );
}