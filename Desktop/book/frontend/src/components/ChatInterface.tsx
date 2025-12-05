import React, { useState } from 'react';
import styles from '../css/chat.module.css'; // Import the CSS module

interface Message {
  text: string;
  sender: 'user' | 'bot';
  sources?: string[];
}

// TODO: Replace with your actual backend URL when deployed
// For local development, it's typically http://localhost:8000
const BACKEND_URL = 'http://localhost:8000'; 

const ChatInterface: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);

  const handleSendMessage = async () => {
    if (input.trim()) {
      const userMessage = { text: input, sender: 'user' as const };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setInput('');
      setLoading(true);

      try {
        const response = await fetch(`${BACKEND_URL}/api/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ query: input }),
        });

        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        setMessages((prevMessages) => [
          ...prevMessages,
          { text: data.answer, sender: 'bot', sources: data.sources },
        ]);
      } catch (error) {
        console.error("Error sending message to backend:", error);
        setMessages((prevMessages) => [
          ...prevMessages,
          { text: "Sorry, I'm having trouble connecting right now. Please try again later.", sender: 'bot' },
        ]);
      } finally {
        setLoading(false);
      }
    }
  };

  return (
    <div className={styles.chatContainer}>
      <div className={styles.messagesContainer}>
        {messages.map((msg, index) => (
          <div key={index} className={`${styles.message} ${msg.sender === 'user' ? styles.userMessage : styles.botMessage}`}>
            <span className={styles.messageContent}>
              {msg.text}
            </span>
            {msg.sources && msg.sources.length > 0 && msg.sender === 'bot' && (
              <div className={styles.messageSource}>
                Source: {msg.sources.join(', ')}
              </div>
            )}
          </div>
        ))}
        {loading && (
          <div className={`${styles.message} ${styles.loadingMessage}`}>
            <span className={styles.loadingContent}>
              Thinking...
            </span>
          </div>
        )}
      </div>
      <div className={styles.inputArea}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => {
            if (e.key === 'Enter') {
              handleSendMessage();
            }
          }}
          placeholder="Ask a question..."
          className={styles.textInput}
          disabled={loading}
          aria-label="Chat message input" // Added for accessibility
        />
        <button
          onClick={handleSendMessage}
          className={styles.sendButton}
          disabled={loading}
          aria-label="Send message" // Added for accessibility
        >
          Send
        </button>
      </div>
    </div>
  );
};

export default ChatInterface;
