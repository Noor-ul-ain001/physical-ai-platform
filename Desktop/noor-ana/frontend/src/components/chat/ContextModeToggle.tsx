import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './ContextModeToggle.module.css';

type ContextMode = 'full_book' | 'selective';

type ContextModeToggleProps = {
  initialMode?: ContextMode;
  onModeChange?: (mode: ContextMode) => void;
  className?: string;
};

export default function ContextModeToggle({ 
  initialMode = 'full_book', 
  onModeChange,
  className 
}: ContextModeToggleProps): JSX.Element {
  const [mode, setMode] = useState<ContextMode>(initialMode);

  const handleModeChange = (newMode: ContextMode) => {
    setMode(newMode);
    onModeChange?.(newMode);
  };

  return (
    <div className={clsx(styles.contextModeToggle, className)}>
      <div className={styles.toggleContainer}>
        <button
          className={clsx(
            styles.toggleButton,
            mode === 'full_book' && styles.active
          )}
          onClick={() => handleModeChange('full_book')}
        >
          Full Book
        </button>
        <button
          className={clsx(
            styles.toggleButton,
            mode === 'selective' && styles.active
          )}
          onClick={() => handleModeChange('selective')}
        >
          Selective
        </button>
        <div className={styles.toggleIndicator} />
      </div>
      <div className={styles.modeDescription}>
        {mode === 'full_book' 
          ? 'Using entire curriculum for context' 
          : 'Using only selected text for context'}
      </div>
    </div>
  );
}