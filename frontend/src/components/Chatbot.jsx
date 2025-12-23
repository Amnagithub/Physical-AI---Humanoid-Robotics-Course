import React, { useState, useRef, useEffect } from 'react';
import './Chatbot.css';

const Chatbot = ({ apiBaseUrl = 'http://localhost:8000' }) => {
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hello! I\'m your Physical AI & Humanoid Robotics Course assistant. How can I help you today?', sender: 'bot' }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [mode, setMode] = useState('full_book'); // 'full_book' or 'selected_text'
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Initialize session on component mount
  useEffect(() => {
    const initializeSession = async () => {
      try {
        const response = await fetch(`${apiBaseUrl}/api/v1/sessions`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            mode: mode
          })
        });

        if (response.ok) {
          const data = await response.json();
          setSessionId(data.id);
        } else {
          console.error('Failed to initialize session');
        }
      } catch (error) {
        console.error('Error initializing session:', error);
      }
    };

    initializeSession();
  }, [apiBaseUrl, mode]);

  const handleSendMessage = async () => {
    if (!inputText.trim() || isLoading || !sessionId) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user'
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Send question to backend
      const response = await fetch(`${apiBaseUrl}/api/v1/sessions/${sessionId}/questions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: inputText,
          selected_text: mode === 'selected_text' ? selectedText : null
        })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.answer,
          sender: 'bot',
          sources: data.sources || []
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorData = await response.json();
        const errorMessage = {
          id: Date.now() + 1,
          text: `Error: ${errorData.detail || 'Failed to get response'}`,
          sender: 'bot'
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: `Error: ${error.message || 'Network error'}`,
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleMode = () => {
    setMode(prevMode => prevMode === 'full_book' ? 'selected_text' : 'full_book');
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Course Assistant</h3>
        <div className="chatbot-mode-toggle">
          <span>Mode: </span>
          <button
            className={`mode-button ${mode === 'full_book' ? 'active' : ''}`}
            onClick={toggleMode}
            disabled={isLoading}
          >
            Full Book
          </button>
          <button
            className={`mode-button ${mode === 'selected_text' ? 'active' : ''}`}
            onClick={toggleMode}
            disabled={isLoading}
          >
            Selected Text
          </button>
        </div>
      </div>

      {mode === 'selected_text' && (
        <div className="selected-text-input">
          <textarea
            value={selectedText}
            onChange={(e) => setSelectedText(e.target.value)}
            placeholder="Paste the text you want to focus on..."
            rows="3"
          />
        </div>
      )}

      <div className="chatbot-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.sender}`}
          >
            <div className="message-text">
              {message.text}
            </div>
            {message.sources && message.sources.length > 0 && (
              <div className="sources">
                <details>
                  <summary>Sources ({message.sources.length})</summary>
                  <ul>
                    {message.sources.map((source, index) => (
                      <li key={index}>
                        {source.section || 'Unknown Section'}
                      </li>
                    ))}
                  </ul>
                </details>
              </div>
            )}
          </div>
        ))}
        {isLoading && (
          <div className="message bot">
            <div className="message-text">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="chatbot-input">
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the course..."
          rows="1"
          disabled={isLoading || !sessionId}
        />
        <button
          onClick={handleSendMessage}
          disabled={!inputText.trim() || isLoading || !sessionId}
          className="send-button"
        >
          Send
        </button>
      </div>
    </div>
  );
};

export default Chatbot;