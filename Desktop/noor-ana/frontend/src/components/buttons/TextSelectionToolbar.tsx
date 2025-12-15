import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import Button from '../common/Button';
import styles from './TextSelectionToolbar.module.css';

interface TextSelectionToolbarProps {
  className?: string;
}

const TextSelectionToolbar: React.FC<TextSelectionToolbarProps> = ({ className }) => {
  const [showToolbar, setShowToolbar] = useState(false);
  const [position, setPosition] = useState({ top: 0, left: 0 });
  const toolbarRef = useRef<HTMLDivElement>(null);
  
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (!selection || selection.toString().trim() === '') {
        setShowToolbar(false);
        return;
      }

      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      const toolbarHeight = 40; // Approximate height of the toolbar
      const scrollTop = window.pageYOffset || document.documentElement.scrollTop;

      setPosition({
        top: rect.top + scrollTop - toolbarHeight - 10, // 10px above selection
        left: rect.left + rect.width / 2 // Centered above selection
      });

      setShowToolbar(true);
    };

    const handleClick = () => {
      const selection = window.getSelection();
      if (!selection || selection.toString().trim() === '') {
        setShowToolbar(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('click', handleClick);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('click', handleClick);
    };
  }, []);

  // Function to handle note taking for selected text
  const handleTakeNote = () => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim() !== '') {
      const selectedText = selection.toString();
      // In a real implementation, this would save the note
      alert(`Noting: "${selectedText}"`);
      setShowToolbar(false);
      window.getSelection()?.empty();
    }
  };

  // Function to search the selected text
  const handleSearch = () => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim() !== '') {
      const selectedText = selection.toString();
      // In a real implementation, this would search for the selected text
      alert(`Searching for: "${selectedText}"`);
      setShowToolbar(false);
      window.getSelection()?.empty();
    }
  };

  if (!showToolbar) {
    return null;
  }

  return (
    <div 
      ref={toolbarRef}
      className={clsx(styles.toolbar, className)}
      style={{
        top: `${position.top}px`,
        left: `${position.left}px`,
        transform: 'translateX(-50%)' // Center the toolbar
      }}
    >
      <Button variant="primary" size="sm" onClick={handleTakeNote}>
        Add Note
      </Button>
      <Button variant="secondary" size="sm" onClick={handleSearch}>
        Search
      </Button>
    </div>
  );
};

export default TextSelectionToolbar;