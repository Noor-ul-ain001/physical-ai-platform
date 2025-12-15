import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './UserAuthButton.module.css';

type UserAuthButtonProps = {
  className?: string;
};

type UserState = {
  isLoggedIn: boolean;
  user?: {
    id: string;
    email: string;
    profileCompleted: boolean;
  };
};

export default function UserAuthButton({ className }: UserAuthButtonProps){
  const [userState, setUserState] = useState<UserState>({ isLoggedIn: false });
  const [menuOpen, setMenuOpen] = useState(false);

  // In a real implementation, this would check authentication status
  useEffect(() => {
    // Simulate checking authentication status
    const checkAuthStatus = () => {
      // In a real app, this would check for auth tokens in local storage or cookies
      const isLoggedIn = localStorage.getItem('isLoggedIn') === 'true';
      
      if (isLoggedIn) {
        setUserState({
          isLoggedIn: true,
          user: {
            id: 'mock-user-id',
            email: 'user@example.com',
            profileCompleted: true
          }
        });
      } else {
        setUserState({ isLoggedIn: false });
      }
    };
    
    checkAuthStatus();
  }, []);

  const handleLogin = () => {
    // In a real implementation, this would redirect to login
    console.log('Redirecting to login');
  };

  const handleSignup = () => {
    // In a real implementation, this would redirect to signup
    console.log('Redirecting to signup');
    window.location.href = '/signup';
  };

  const handleLogout = () => {
    // In a real implementation, this would logout the user
    localStorage.setItem('isLoggedIn', 'false');
    setUserState({ isLoggedIn: false });
    setMenuOpen(false);
  };

  const toggleMenu = () => {
    setMenuOpen(!menuOpen);
  };

  return (
    <div className={clsx(styles.authButtonContainer, className)}>
      {userState.isLoggedIn ? (
        <div className={styles.loggedInContainer}>
          <button 
            className={clsx(styles.userButton, styles.loggedIn)}
            onClick={toggleMenu}
          >
            <span className={styles.userInitial}>
              {userState.user?.email.charAt(0).toUpperCase() || 'U'}
            </span>
            <span className={styles.userName}>
              {userState.user?.email.split('@')[0]}
            </span>
            <span className={styles.dropdownArrow}>▼</span>
          </button>
          
          {menuOpen && (
            <div className={styles.dropdownMenu}>
              <a href="/dashboard" className={styles.menuItem}>Dashboard</a>
              <a href="/profile" className={styles.menuItem}>Profile</a>
              <a href="/settings" className={styles.menuItem}>Settings</a>
              <button 
                onClick={handleLogout} 
                className={clsx(styles.menuItem, styles.logoutButton)}
              >
                Logout
              </button>
            </div>
          )}
        </div>
      ) : (
        <div className={styles.loggedOutContainer}>
          <button 
            className={clsx(styles.authButton, styles.loginButton)}
            onClick={handleLogin}
          >
            Login
          </button>
          <button 
            className={clsx(styles.authButton, styles.signupButton)}
            onClick={handleSignup}
          >
            Sign Up
          </button>
        </div>
      )}
    </div>
  );
}