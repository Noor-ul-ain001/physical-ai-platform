import React, { useState, useEffect, useRef, FormEvent, ChangeEvent } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';
import axios from 'axios';

// Import only essential icons
import { FiSend, FiPaperclip, FiMic, FiTrash2, FiCopy, FiUser, FiMenu, FiX } from 'react-icons/fi';
import { RiRobotLine } from 'react-icons/ri';

interface ChatMessage {
  id: string;
  sender: 'user' | 'bot';
  text: string;
  timestamp: Date;
  isTyping?: boolean;
}

interface SuggestedQuestion {
  id: string;
  text: string;
}

const Chatbot = () => {
  const [messages, setMessages] = useState<ChatMessage[]>([
    {
      id: '1',
      sender: 'bot',
      text: 'Hello! I\'m your textbook assistant. How can I help you today?',
      timestamp: new Date(),
    },
  ]);
  const [input, setInput] = useState<string>('');
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [isTyping, setIsTyping] = useState<boolean>(false);
  const [sessionId] = useState<string>(
    `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`
  );
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState<boolean>(false);
  
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Simple suggested questions
  const suggestedQuestions: SuggestedQuestion[] = [
    { id: 'q1', text: 'Summarize this' },
    { id: 'q2', text: 'Explain this' },
    { id: 'q3', text: 'Give example' },
    { id: 'q4', text: 'Key points?' },
  ];

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Close mobile menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (chatContainerRef.current && !chatContainerRef.current.contains(e.target as Node)) {
        setIsMobileMenuOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  // Handle text selection
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim().length > 0) {
        const selectedText = selection.toString().trim();
        if (selectedText.length > 5 && selectedText.length < 1000) {
          setSelectedText(selectedText);
          
          const botMessage: ChatMessage = {
            id: (Date.now() + 1).toString(),
            sender: 'bot',
            text: `I noticed you selected some text. How can I help you with it?`,
            timestamp: new Date(),
          };
    
          setMessages(prev => [...prev, botMessage]);
        }
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  const handleInputChange = (e: ChangeEvent<HTMLTextAreaElement>) => {
    setInput(e.target.value);
    // Auto-resize textarea
    if (inputRef.current) {
      inputRef.current.style.height = 'auto';
      inputRef.current.style.height = `${Math.min(inputRef.current.scrollHeight, 120)}px`;
    }
  };

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    if (input.trim() === '') return;

    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      sender: 'user',
      text: input,
      timestamp: new Date(),
    };
    
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    
    // Reset textarea height
    if (inputRef.current) {
      inputRef.current.style.height = 'auto';
    }

    setIsTyping(true);

    try {
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
      let response;

      if (selectedText) {
        response = await axios.post(`${backendUrl}/selected-chat`, {
          text: selectedText,
          query: userMessage.text,
          session_id: sessionId,
        });
      } else {
        response = await axios.post(`${backendUrl}/chat`, {
          query: userMessage.text,
          session_id: sessionId,
        });
      }

      // Clear selection after sending
      if (selectedText) {
        setSelectedText(null);
      }

      // Simulate typing delay
      await new Promise(resolve => setTimeout(resolve, 500));

      const botMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        sender: 'bot',
        text: response.data.response,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);
      setIsTyping(false);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        sender: 'bot',
        text: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
      setIsTyping(false);
    }
  };

  const handleSuggestedQuestion = (question: string) => {
    setInput(question);
    if (inputRef.current) {
      inputRef.current.focus();
    }
    setIsMobileMenuOpen(false); // Close menu on mobile after selecting
  };

  const handleFileUpload = async (e: ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    const allowedTypes = [
      'text/plain',
      'application/pdf',
      'application/msword',
      'application/vnd.openxmlformats-officedocument.wordprocessingml.document'
    ];
    
    if (!allowedTypes.includes(file.type) && !file.name.match(/\.(txt|pdf|doc|docx)$/i)) {
      alert('Please upload a text file (txt), PDF, or Word document');
      return;
    }

    const loadingMessage: ChatMessage = {
      id: 'upload-' + Date.now(),
      sender: 'bot',
      text: `📤 Uploading "${file.name}"...`,
      timestamp: new Date(),
    };
    setMessages(prev => [...prev, loadingMessage]);

    const formData = new FormData();
    formData.append('file', file);
    formData.append('session_id', sessionId);

    try {
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
      await axios.post(`${backendUrl}/upload`, formData, {
        headers: {
          'Content-Type': 'multipart/form-data',
        },
      });

      const successMessage: ChatMessage = {
        id: 'upload-success-' + Date.now(),
        sender: 'bot',
        text: `✅ Successfully uploaded "${file.name}". You can now ask questions about this document.`,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, successMessage]);
    } catch (error) {
      console.error('Upload error:', error);
      const errorMessage: ChatMessage = {
        id: 'upload-error-' + Date.now(),
        sender: 'bot',
        text: '❌ Failed to upload document. Please try again.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    }
  };

  const handleClearChat = () => {
    if (window.confirm('Clear chat history?')) {
      setMessages([
        {
          id: '1',
          sender: 'bot',
          text: 'Chat cleared. How can I help you today?',
          timestamp: new Date(),
        },
      ]);
      setSelectedText(null);
    }
    setIsMobileMenuOpen(false);
  };

  const handleCopyMessage = (text: string) => {
    navigator.clipboard.writeText(text)
      .then(() => {
        // Optional: Show a brief "Copied!" message
      })
      .catch(err => console.error('Failed to copy:', err));
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e as any);
    }
  };

  const formatTime = (date: Date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div ref={chatContainerRef} className={styles.chatbotContainer}>
      {/* Chat Header */}
      <div className={styles.chatHeader}>
        <div className={styles.headerLeft}>
          <div className={styles.botAvatar}>
            <RiRobotLine size={20} />
          </div>
          <div className={styles.headerInfo}>
            <h3 className={styles.chatTitle}>Textbook Assistant</h3>
            <span className={styles.chatStatus}>
              {isTyping ? 'Typing...' : 'Online'}
            </span>
          </div>
        </div>
        
        {/* Mobile Menu Button */}
        <button 
          className={styles.mobileMenuButton}
          onClick={() => setIsMobileMenuOpen(!isMobileMenuOpen)}
          aria-label={isMobileMenuOpen ? "Close menu" : "Open menu"}
        >
          {isMobileMenuOpen ? <FiX size={20} /> : <FiMenu size={20} />}
        </button>

        {/* Desktop Actions */}
        <div className={styles.headerActions}>
          <button 
            className={styles.headerButton}
            onClick={handleClearChat}
            title="Clear chat"
          >
            <FiTrash2 size={16} />
          </button>
        </div>

        {/* Mobile Menu Dropdown */}
        {isMobileMenuOpen && (
          <div className={styles.mobileMenu}>
            <button 
              className={styles.mobileMenuItem}
              onClick={handleClearChat}
            >
              <FiTrash2 size={16} />
              <span>Clear Chat</span>
            </button>
            <button 
              className={styles.mobileMenuItem}
              onClick={() => {
                fileInputRef.current?.click();
                setIsMobileMenuOpen(false);
              }}
            >
              <FiPaperclip size={16} />
              <span>Upload File</span>
            </button>
            <div className={styles.mobileSuggestedQuestions}>
              <p>Quick Questions:</p>
              {suggestedQuestions.map((question) => (
                <button
                  key={question.id}
                  className={styles.mobileSuggestionButton}
                  onClick={() => handleSuggestedQuestion(question.text)}
                >
                  {question.text}
                </button>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* Selected Text Banner */}
      {selectedText && (
        <div className={styles.selectedTextBanner}>
          <div className={styles.selectedTextContent}>
            <span className={styles.bannerLabel}>
              📝 Selected Text
            </span>
            <p className={styles.selectedTextPreview}>
              "{selectedText.length > 60 ? selectedText.substring(0, 60) + '...' : selectedText}"
            </p>
          </div>
          <button 
            className={styles.bannerButton}
            onClick={() => setSelectedText(null)}
            title="Clear selection"
          >
            ✕
          </button>
        </div>
      )}

      {/* Messages Container */}
      <div className={styles.messagesContainer}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={clsx(
              styles.messageWrapper,
              message.sender === 'user' ? styles.userWrapper : styles.botWrapper
            )}
          >
            <div className={clsx(styles.messageAvatar, message.sender === 'user' ? styles.userAvatar : styles.botAvatar)}>
              {message.sender === 'user' ? (
                <FiUser size={14} />
              ) : (
                <RiRobotLine size={14} />
              )}
            </div>
            <div className={styles.messageContent}>
              <div className={styles.messageHeader}>
                <span className={styles.senderName}>
                  {message.sender === 'user' ? 'You' : 'Assistant'}
                </span>
                <span className={styles.messageTime}>
                  {formatTime(message.timestamp)}
                </span>
              </div>
              <div className={clsx(
                styles.messageBubble,
                message.sender === 'user' ? styles.userBubble : styles.botBubble
              )}>
                {message.isTyping ? (
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                ) : (
                  <>
                    <div className={styles.messageTextContainer}>
                      <p className={styles.messageText}>{message.text}</p>
                    </div>
                    <button
                      className={styles.copyButton}
                      onClick={() => handleCopyMessage(message.text)}
                      title="Copy message"
                    >
                      <FiCopy size={12} />
                    </button>
                  </>
                )}
              </div>
            </div>
          </div>
        ))}
        
        {isTyping && (
          <div className={styles.messageWrapper}>
            <div className={clsx(styles.messageAvatar, styles.botAvatar)}>
              <RiRobotLine size={14} />
            </div>
            <div className={styles.messageContent}>
              <div className={styles.messageHeader}>
                <span className={styles.senderName}>Assistant</span>
              </div>
              <div className={clsx(styles.messageBubble, styles.botBubble)}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          </div>
        )}
        
        <div ref={messagesEndRef} className={styles.scrollAnchor} />
      </div>

      {/* Suggested Questions (Desktop only) */}
      {messages.length <= 3 && (
        <div className={styles.suggestedQuestions}>
          <p className={styles.suggestionsTitle}>Try asking:</p>
          <div className={styles.questionsGrid}>
            {suggestedQuestions.map((question) => (
              <button
                key={question.id}
                className={styles.suggestionButton}
                onClick={() => handleSuggestedQuestion(question.text)}
              >
                {question.text}
              </button>
            ))}
          </div>
        </div>
      )}

      {/* Input Area */}
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <div className={styles.inputWrapper}>
          <textarea
            ref={inputRef}
            value={input}
            onChange={handleInputChange}
            onKeyDown={handleKeyDown}
            placeholder={
              selectedText 
                ? `Ask about selected text...` 
                : 'Type your question...'
            }
            className={styles.inputField}
            rows={1}
          />
          
          <div className={styles.inputActions}>
            <button
              type="button"
              className={styles.actionButton}
              onClick={() => fileInputRef.current?.click()}
              title="Attach document"
            >
              <FiPaperclip size={16} />
              <input
                ref={fileInputRef}
                type="file"
                accept=".txt,.pdf,.doc,.docx"
                onChange={handleFileUpload}
                className={styles.fileInput}
              />
            </button>
            
            <button
              type="submit"
              className={styles.sendButton}
              disabled={!input.trim()}
              title="Send message"
            >
              <FiSend size={16} />
            </button>
          </div>
        </div>
      </form>
    </div>
  );
};

export default Chatbot;