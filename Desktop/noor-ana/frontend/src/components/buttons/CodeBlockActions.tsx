import React, { useState } from 'react';
import clsx from 'clsx';
import Button from '../common/Button';
import styles from './CodeBlockActions.module.css';

interface CodeBlockActionsProps {
  code: string;
  language?: string;
  className?: string;
}

const CodeBlockActions: React.FC<CodeBlockActionsProps> = ({ 
  code, 
  language = 'text',
  className 
}) => {
  const [copied, setCopied] = useState(false);
  const [expanded, setExpanded] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(code)
      .then(() => {
        setCopied(true);
        setTimeout(() => setCopied(false), 2000);
      })
      .catch(err => {
        console.error('Failed to copy: ', err);
      });
  };

  const handleExpand = () => {
    setExpanded(!expanded);
  };

  return (
    <div className={clsx(styles.actionsContainer, className)}>
      <Button 
        variant="ghost" 
        size="sm" 
        onClick={handleCopy}
        aria-label={copied ? "Copied!" : "Copy code"}
      >
        {copied ? '✓ Copied!' : 'Copy'}
      </Button>
      
      {code.split('\n').length > 10 && (
        <Button 
          variant="ghost" 
          size="sm" 
          onClick={handleExpand}
          aria-label={expanded ? "Collapse code" : "Expand code"}
        >
          {expanded ? 'Collapse' : 'Expand'}
        </Button>
      )}
    </div>
  );
};

export default CodeBlockActions;