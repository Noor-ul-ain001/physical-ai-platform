import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './InteractiveCodeBlock.module.css';

type InteractiveCodeBlockProps = {
  children: React.ReactNode;
  language?: string;
  title?: string;
};

export default function InteractiveCodeBlock({ 
  children, 
  language = 'python',
  title = 'Code Example'
}: InteractiveCodeBlockProps): JSX.Element {
  const [copied, setCopied] = useState(false);

  const copyToClipboard = () => {
    const codeElement = document.querySelector(`.${styles.codeBlock}`);
    if (codeElement) {
      navigator.clipboard.writeText((codeElement as HTMLElement).innerText)
        .then(() => {
          setCopied(true);
          setTimeout(() => setCopied(false), 2000);
        });
    }
  };

  return (
    <div className={clsx('interactive-code-block', styles.interactiveCodeBlock)}>
      <div className={styles.codeHeader}>
        <span className={styles.codeTitle}>{title}</span>
        <button 
          className={clsx('copy-button', styles.copyButton)} 
          onClick={copyToClipboard}
        >
          {copied ? '✓ Copied!' : 'Copy'}
        </button>
      </div>
      <div className={clsx(styles.codeBlock, 'language-' + language)}>
        {children}
      </div>
    </div>
  );
}