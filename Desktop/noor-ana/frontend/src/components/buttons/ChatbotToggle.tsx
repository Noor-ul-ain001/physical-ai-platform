import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './ChatbotToggle.module.css';
import ChatbotWidget from '../chat/ChatbotWidget';

type ChatbotToggleProps = {
  className?: string;
};

export default function ChatbotToggle({ className }: ChatbotToggleProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [notifications, setNotifications] = useState(0);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      // Reset notifications when opening
      setNotifications(0);
    }
  };

  // This would be called when a new message arrives
  const addNotification = () => {
    if (!isOpen) {
      setNotifications(prev => prev + 1);
    }
  };

  return (
    <div className={clsx(styles.chatbotToggleContainer, className)}>
      {isOpen && (
        <div className={styles.chatbotWidgetOverlay}>
          <div className={styles.chatbotWidget}>
            <div className={styles.widgetHeader}>
              <h3>Physical AI Assistant</h3>
              <button 
                className={styles.closeButton} 
                onClick={toggleChat}
                aria-label="Close chat"
              >
                ×
              </button>
            </div>
            <ChatbotWidget />
          </div>
        </div>
      )}
      
      <button
        className={clsx(
          styles.toggleButton,
          isOpen && styles.openButton
        )}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        <div className={styles.buttonContent}>
          <span className={styles.icon}>🤖</span>
          {!isOpen && notifications > 0 && (
            <span className={styles.notificationBadge}>
              {notifications}
            </span>
          )}
        </div>
      </button>
    </div>
  );
}