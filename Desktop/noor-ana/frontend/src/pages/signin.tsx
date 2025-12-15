import React, { useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './signin.module.css';

type FormData = {
  email: string;
  password: string;
  rememberMe: boolean;
};

export default function SigninPage(): JSX.Element {
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    rememberMe: false
  });
  const [errors, setErrors] = useState<Partial<FormData>>({});

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value, type, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
    
    // Clear error when user starts typing
    if (errors[name as keyof FormData]) {
      setErrors(prev => ({
        ...prev,
        [name]: undefined
      }));
    }
  };

  const validate = (): boolean => {
    const newErrors: Partial<FormData> = {};
    
    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
      newErrors.email = 'Email address is invalid';
    }
    
    if (!formData.password) {
      newErrors.password = 'Password is required';
    }
    
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!validate()) {
      return;
    }
    
    // In a real implementation, this would call the backend API
    console.log('Signing in user:', formData);
    
    // Simulate login
    localStorage.setItem('isLoggedIn', 'true');
    window.location.href = '/';
  };

  return (
    <Layout title="Sign In" description="Sign in to your Physical AI & Humanoid Robotics account">
      <main className={styles.main}>
        <div className={styles.container}>
          <div className={styles.formContainer}>
            <h1>Sign In</h1>
            <p className={styles.subtitle}>
              Access your Physical AI & Humanoid Robotics account
            </p>
            
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
                  placeholder="Your password"
                />
                {errors.password && <span className={styles.error}>{errors.password}</span>}
              </div>
              
              <div className={styles.checkboxGroup}>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    name="rememberMe"
                    checked={formData.rememberMe}
                    onChange={handleChange}
                    className={styles.checkbox}
                  />
                  <span>Remember me</span>
                </label>
                
                <a href="/reset-password" className={styles.forgotPassword}>
                  Forgot password?
                </a>
              </div>
              
              <button 
                type="submit" 
                className={styles.submitButton}
              >
                Sign In
              </button>
            </form>
            
            <div className={styles.signupLink}>
              Don't have an account? <a href="/signup">Sign up here</a>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}