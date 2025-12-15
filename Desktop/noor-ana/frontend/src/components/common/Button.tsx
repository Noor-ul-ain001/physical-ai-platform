import React from 'react';
import clsx from 'clsx';
import { motion } from 'framer-motion';
import styles from './Button.module.css';

type ButtonVariant = 'primary' | 'secondary' | 'ghost' | 'destructive';
type ButtonSize = 'sm' | 'md' | 'lg';

interface ButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: ButtonVariant;
  size?: ButtonSize;
  isLoading?: boolean;
  asChild?: boolean;
  children: React.ReactNode;
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ 
    className, 
    variant = 'primary', 
    size = 'md', 
    isLoading = false, 
    disabled, 
    children, 
    asChild = false,
    ...props 
  }, ref) => {
    const isDisabled = disabled || isLoading;

    if (asChild && React.isValidElement(children)) {
      return React.cloneElement(children, {
        className: clsx(
          styles.button,
          styles[variant],
          styles[size],
          isDisabled && styles.disabled,
          className
        ),
        disabled: isDisabled,
        ref,
        ...props
      } as React.ButtonHTMLAttributes<HTMLButtonElement>);
    }

    return (
      <motion.button
        ref={ref}
        className={clsx(
          styles.button,
          styles[variant],
          styles[size],
          isDisabled && styles.disabled,
          className
        )}
        disabled={isDisabled}
        whileHover={isDisabled ? {} : { scale: 1.03 }}
        whileTap={isDisabled ? {} : { scale: 0.98 }}
        {...props}
      >
        {isLoading && (
          <span className={styles.spinner} aria-hidden="true" />
        )}
        {isLoading ? (
          <span className={styles.loadingText}>Loading...</span>
        ) : (
          children
        )}
      </motion.button>
    );
  }
);

Button.displayName = 'Button';

export default Button;