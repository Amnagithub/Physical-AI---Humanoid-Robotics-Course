import React, { useState } from 'react';
import './FloatingChatButton.css';

const FloatingChatButton = ({ apiBaseUrl = 'http://localhost:8000' }) => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hello! I\'m your Physical AI & Humanoid Robotics Course assistant. How can I help you today?', sender: 'bot' }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [mode, setMode] = useState('full_book');

  // Initialize session when chat is opened
  React.useEffect(() => {
    const initializeSession = async () => {
      if (isChatOpen && !sessionId) {
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
          }
        } catch (error) {
          console.error('Error initializing session:', error);
        }
      }
    };

    initializeSession();
  }, [isChatOpen, sessionId, apiBaseUrl, mode]);

  const handleSendMessage = async () => {
    if (!inputText.trim() || isLoading || !sessionId) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      const response = await fetch(`${apiBaseUrl}/api/v1/sessions/${sessionId}/questions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: inputText,
          selected_text: mode === 'selected_text' ? '' : null
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

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  const closeChat = () => {
    setIsChatOpen(false);
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className="floating-chat-button"
        onClick={toggleChat}
        aria-label="Open chat"
      >
        <div className="chat-icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H16.71C16.567 17.614 16.338 18.199 16.03 18.73L16.03 18.73L16.9 19.6C17.0892 19.7892 17.193 20.0443 17.193 20.31C17.193 20.5757 17.0892 20.8308 16.9 21C16.7108 21.1892 16.4557 21.293 16.19 21.293C15.9243 21.293 15.6692 21.1892 15.48 21L14.25 19.77C13.69 20.04 13.08 20.22 12.44 20.3C12.27 20.32 12.1 20.33 11.93 20.33C10.43 20.33 9.01 20.04 7.7 19.5C5.9 18.7 4.5 17.3 3.7 15.5C2.9 13.7 2.5 11.7 2.5 9.6C2.5 7.5 2.9 5.5 3.7 3.7C4.5 1.9 5.8 0.5 7.6 0.5C9.4 0.5 11.1 1.1 12.4 2.3C13.7 1.1 15.5 0.5 17.3 0.5C19.1 0.5 20.8 1.1 22.1 2.3C23.4 3.5 24 5.2 24 7C24 8.8 23.6 10.6 22.8 12.4C22 14.2 21.2 15.5 21 15Z" fill="currentColor"/>
          </svg>
        </div>
        {isLoading && <div className="typing-indicator-dot"></div>}
      </button>

      {/* Chat Modal */}
      {isChatOpen && (
        <div className="chat-modal-overlay" onClick={closeChat}>
          <div className="chat-modal" onClick={(e) => e.stopPropagation()}>
            <div className="chat-header">
              <h3>Course Assistant</h3>
              <button className="close-button" onClick={closeChat}>
                <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M15 5L5 15M5 5L15 15" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>

            <div className="chat-mode-toggle">
              <button
                className={`mode-button ${mode === 'full_book' ? 'active' : ''}`}
                onClick={() => setMode('full_book')}
              >
                Full Book
              </button>
              <button
                className={`mode-button ${mode === 'selected_text' ? 'active' : ''}`}
                onClick={() => setMode('selected_text')}
              >
                Selected Text
              </button>
            </div>

            <div className="chat-messages">
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
            </div>

            <div className="chat-input">
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
                <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M18.3333 9.16667L1.66667 1.66667L9.16667 10L1.66667 18.3333L18.3333 10.8333L14.1667 10L18.3333 9.16667Z" fill="currentColor"/>
                </svg>
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatButton;