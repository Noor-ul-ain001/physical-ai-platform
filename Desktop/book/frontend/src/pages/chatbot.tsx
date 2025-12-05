import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../components/ChatInterface'; // Import the ChatInterface component

function Chatbot() {
  return (
    <Layout
      title="Chat with the Book"
      description="An AI Chatbot that can answer questions about the Book of AI-Driven Development"
    >
      <main>
        <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
          <h1>Chat with the Book</h1>
          <p>Ask me anything about the content of "The Book of AI-Driven Development".</p>
          <ChatInterface /> {/* Render the ChatInterface component */}
        </div>
      </main>
    </Layout>
  );
}

export default Chatbot;
