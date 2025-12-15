import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import { ChatAPI } from '../../lib/api';
import styles from './ChatbotWidget.module.css';

type Message = {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  citations?: Array<{
    source: string;
    excerpt: string;
    similarity: number;
  }>;
};

type ChatbotWidgetProps = {
  initialMessages?: Message[];
  onSendMessage?: (message: string) => void;
  className?: string;
};

export default function ChatbotWidget({ 
  initialMessages = [],
  onSendMessage,
  className
}: ChatbotWidgetProps): JSX.Element {
  const [messages, setMessages] = useState<Message[]>(initialMessages);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the real backend API
      const response = await ChatAPI.sendMessage({
        query: inputValue,
        top_k: 5
      });

      if (response.error) {
        throw new Error(response.error);
      }

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: response.data!.answer,
        role: 'assistant',
        timestamp: new Date(),
        citations: response.data!.citations
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: 'Sorry, I encountered an error processing your request. ' +
                 (error instanceof Error ? error.message : 'Please try again.'),
        role: 'assistant',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className={clsx(styles.chatContainer, className)}>
      <div className={styles.chatHeader}>
        <h3>Physical AI Assistant</h3>
        <p>Ask me anything about the curriculum!</p>
      </div>
      
      <div className={styles.messagesContainer}>
        {messages.length === 0 ? (
          <div className={styles.welcomeMessage}>
            <p>Hello! I'm your Physical AI & Humanoid Robotics teaching assistant.</p>
            <p>Ask me about ROS 2, simulation, NVIDIA Isaac Sim, or VLA models.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div 
              key={message.id} 
              className={clsx(
                styles.message, 
                styles[message.role],
                message.role === 'user' && styles.userMessage,
                message.role === 'assistant' && styles.assistantMessage
              )}
            >
              <div className={styles.messageContent}>
                {message.content}
              </div>
              {message.citations && message.citations.length > 0 && (
                <div className={styles.citations}>
                  <h4>Sources:</h4>
                  <ul>
                    {message.citations.map((citation, index) => (
                      <li key={index}>
                        <a href={`/docs/${citation.source}`} target="_blank" rel="noopener noreferrer">
                          {citation.source}
                        </a>
                        <p className={styles.citationExcerpt}>{citation.excerpt}</p>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          ))
        )}
        {isLoading && (
          <div className={clsx(styles.message, styles.assistantMessage)}>
            <div className={styles.typingIndicator}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>
      
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the curriculum..."
          className={styles.input}
          disabled={isLoading}
        />
        <button 
          type="submit" 
          className={styles.sendButton}
          disabled={isLoading || !inputValue.trim()}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
}