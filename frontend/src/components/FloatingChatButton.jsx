import React, { useState, useRef, useEffect, useCallback } from 'react';
import './FloatingChatButton.css';

// Demo responses for when backend is not available
const DEMO_RESPONSES = [
  {
    text: "ROS 2 (Robot Operating System 2) is the nervous system of modern robots. It provides the communication infrastructure that allows different parts of a robot to talk to each other, similar to how neurons communicate in biological systems.",
    sources: ["Module 1: ROS 2 Introduction", "Chapter 1.2: Core Concepts"],
    confidence: 0.92
  },
  {
    text: "Humanoid robots use a combination of sensors (proprioceptive and exteroceptive), actuators, and control systems. The sensory data flows through ROS 2 topics, while commands are sent via services and actions.",
    sources: ["Module 2: Humanoid Architecture", "Chapter 2.1: Sensor Systems"],
    confidence: 0.88
  },
  {
    text: "The Digital Twin concept allows you to simulate your robot in Gazebo or Unity before deploying to real hardware. This is crucial for testing control algorithms safely and efficiently.",
    sources: ["Module 3: Digital Twin", "Chapter 3.1: Simulation Basics"],
    confidence: 0.85
  },
  {
    text: "Vision-Language-Action (VLA) models are a breakthrough in robotics AI. They combine visual perception, natural language understanding, and action planning into unified systems that can understand and execute complex tasks.",
    sources: ["Module 4: VLA Models", "Chapter 4.2: Architecture Overview"],
    confidence: 0.90
  },
  {
    text: "In ROS 2, nodes communicate through topics (pub/sub), services (request/response), and actions (long-running tasks with feedback). This forms the backbone of the robotic nervous system.",
    sources: ["Module 1: ROS 2 Communication", "Chapter 1.4: Communication Patterns"],
    confidence: 0.95
  }
];

const FloatingChatButton = ({ apiBaseUrl = 'http://localhost:8000' }) => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hi! I'm your AI assistant for the Physical AI & Humanoid Robotics Course. Ask me anything about ROS 2, robotics concepts, or course materials!",
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [mode, setMode] = useState('full_book');
  const [connectionStatus, setConnectionStatus] = useState('idle'); // idle, connecting, connected, demo, error
  const [retryCount, setRetryCount] = useState(0);
  const [isDemoMode, setIsDemoMode] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const maxRetries = 2;

  // Auto-scroll to bottom when new messages arrive
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Focus input when chat opens and connected
  useEffect(() => {
    if (isChatOpen && (sessionId || isDemoMode) && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isChatOpen, sessionId, isDemoMode]);

  // Initialize session with retry logic
  const initializeSession = useCallback(async () => {
    if (connectionStatus === 'connecting') return;

    setConnectionStatus('connecting');

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 5000); // 5s timeout

      const response = await fetch(`${apiBaseUrl}/api/v1/sessions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ mode }),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        const data = await response.json();
        setSessionId(data.id);
        setConnectionStatus('connected');
        setRetryCount(0);
        setIsDemoMode(false);
      } else {
        throw new Error(`Server responded with ${response.status}`);
      }
    } catch (error) {
      console.error('Session initialization error:', error);

      if (retryCount < maxRetries) {
        setRetryCount(prev => prev + 1);
        setConnectionStatus('idle');
        // Retry after delay
        setTimeout(() => {
          if (isChatOpen) initializeSession();
        }, 1500 * (retryCount + 1));
      } else {
        // Switch to demo mode
        setConnectionStatus('demo');
        setIsDemoMode(true);
        setMessages(prev => [
          ...prev,
          {
            id: Date.now(),
            text: "Running in demo mode - I'll provide sample responses about the course content. For full functionality, please start the backend server.",
            sender: 'bot',
            isSystem: true,
            timestamp: new Date()
          }
        ]);
      }
    }
  }, [apiBaseUrl, mode, connectionStatus, retryCount, isChatOpen]);

  // Trigger session initialization when chat opens
  useEffect(() => {
    if (isChatOpen && !sessionId && !isDemoMode && connectionStatus === 'idle') {
      initializeSession();
    }
  }, [isChatOpen, sessionId, isDemoMode, connectionStatus, initializeSession]);

  // Demo mode response generator
  const getDemoResponse = (question) => {
    const lowerQuestion = question.toLowerCase();

    // Find best matching response based on keywords
    if (lowerQuestion.includes('ros') || lowerQuestion.includes('nervous')) {
      return DEMO_RESPONSES[0];
    } else if (lowerQuestion.includes('humanoid') || lowerQuestion.includes('sensor')) {
      return DEMO_RESPONSES[1];
    } else if (lowerQuestion.includes('digital twin') || lowerQuestion.includes('simulation') || lowerQuestion.includes('gazebo')) {
      return DEMO_RESPONSES[2];
    } else if (lowerQuestion.includes('vla') || lowerQuestion.includes('vision') || lowerQuestion.includes('language')) {
      return DEMO_RESPONSES[3];
    } else if (lowerQuestion.includes('topic') || lowerQuestion.includes('service') || lowerQuestion.includes('action') || lowerQuestion.includes('communication')) {
      return DEMO_RESPONSES[4];
    }

    // Random response for other questions
    return DEMO_RESPONSES[Math.floor(Math.random() * DEMO_RESPONSES.length)];
  };

  const handleSendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputText;
    setInputText('');
    setIsLoading(true);

    // Demo mode handling
    if (isDemoMode) {
      setTimeout(() => {
        const demoResponse = getDemoResponse(currentInput);
        const botMessage = {
          id: Date.now() + 1,
          text: demoResponse.text,
          sender: 'bot',
          sources: demoResponse.sources,
          confidence: demoResponse.confidence,
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botMessage]);
        setIsLoading(false);
      }, 1000 + Math.random() * 1000); // Simulate network delay
      return;
    }

    // Real API handling
    if (!sessionId) {
      setIsLoading(false);
      return;
    }

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 60000);

      const response = await fetch(`${apiBaseUrl}/api/v1/sessions/${sessionId}/questions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: currentInput,
          selected_text: mode === 'selected_text' ? '' : null
        }),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.answer.content,
          sender: 'bot',
          sources: data.answer.sources || [],
          confidence: data.answer.confidence_score,
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorData = await response.json().catch(() => ({}));
        const errorMessage = {
          id: Date.now() + 1,
          text: `Sorry, I encountered an error: ${errorData.detail || 'Unable to process your request. Please try again.'}`,
          sender: 'bot',
          isError: true,
          timestamp: new Date()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      let errorText = 'Please check your connection and try again.';
      if (error.name === 'AbortError') {
        errorText = 'Request timed out. The server might be busy, please try again.';
      } else if (error.message) {
        errorText = error.message;
      }

      const errorMessage = {
        id: Date.now() + 1,
        text: `Connection error: ${errorText}`,
        sender: 'bot',
        isError: true,
        timestamp: new Date()
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

  const handleRetryConnection = () => {
    setRetryCount(0);
    setConnectionStatus('idle');
    setSessionId(null);
    setIsDemoMode(false);
    initializeSession();
  };

  const clearChat = () => {
    setMessages([
      {
        id: Date.now(),
        text: "Chat cleared! How can I help you?",
        sender: 'bot',
        timestamp: new Date()
      }
    ]);
    if (!isDemoMode) {
      setSessionId(null);
      setConnectionStatus('idle');
      setRetryCount(0);
    }
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  const getStatusText = () => {
    switch (connectionStatus) {
      case 'connecting':
        return `Connecting${retryCount > 0 ? ` (${retryCount}/${maxRetries})` : '...'}`;
      case 'connected':
        return 'Online';
      case 'demo':
        return 'Demo Mode';
      case 'error':
        return 'Connection failed';
      default:
        return 'Connecting...';
    }
  };

  const getStatusClass = () => {
    switch (connectionStatus) {
      case 'connected':
        return 'online';
      case 'demo':
        return 'demo';
      case 'error':
        return 'error';
      default:
        return 'connecting';
    }
  };

  const isInputEnabled = connectionStatus === 'connected' || connectionStatus === 'demo';

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`floating-chat-btn ${isChatOpen ? 'active' : ''}`}
        onClick={toggleChat}
        aria-label={isChatOpen ? 'Close chat' : 'Open chat'}
      >
        <div className="chat-btn-icon">
          {isChatOpen ? (
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M6 18L18 6M6 6l12 12" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          ) : (
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M21 11.5a8.38 8.38 0 01-.9 3.8 8.5 8.5 0 01-7.6 4.7 8.38 8.38 0 01-3.8-.9L3 21l1.9-5.7a8.38 8.38 0 01-.9-3.8 8.5 8.5 0 014.7-7.6 8.38 8.38 0 013.8-.9h.5a8.48 8.48 0 018 8v.5z" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          )}
        </div>
        {isLoading && <span className="loading-pulse"></span>}
      </button>

      {/* Chat Window */}
      {isChatOpen && (
        <div className="chat-container">
          <div className="chat-window">
            {/* Header */}
            <div className="chat-window-header">
              <div className="header-info">
                <div className="bot-avatar">
                  <svg viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2a2 2 0 012 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 017 7h1a1 1 0 011 1v3a1 1 0 01-1 1h-1v1a2 2 0 01-2 2H5a2 2 0 01-2-2v-1H2a1 1 0 01-1-1v-3a1 1 0 011-1h1a7 7 0 017-7h1V5.73c-.6-.34-1-.99-1-1.73a2 2 0 012-2zM7.5 13a1.5 1.5 0 100 3 1.5 1.5 0 000-3zm9 0a1.5 1.5 0 100 3 1.5 1.5 0 000-3zM12 9a5 5 0 00-5 5v1h10v-1a5 5 0 00-5-5z"/>
                  </svg>
                </div>
                <div className="header-text">
                  <h3>Course Assistant</h3>
                  <span className={`status ${getStatusClass()}`}>
                    {getStatusText()}
                    {(connectionStatus === 'error' || connectionStatus === 'demo') && (
                      <button className="retry-btn" onClick={handleRetryConnection}>
                        {connectionStatus === 'demo' ? 'Reconnect' : 'Retry'}
                      </button>
                    )}
                  </span>
                </div>
              </div>
              <div className="header-actions">
                <button
                  className="header-btn"
                  onClick={clearChat}
                  title="Clear chat"
                >
                  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M3 6h18M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a2 2 0 012-2h4a2 2 0 012 2v2" strokeLinecap="round" strokeLinejoin="round"/>
                  </svg>
                </button>
                <button
                  className="header-btn close"
                  onClick={closeChat}
                  title="Close chat"
                >
                  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M6 18L18 6M6 6l12 12" strokeLinecap="round" strokeLinejoin="round"/>
                  </svg>
                </button>
              </div>
            </div>

            {/* Mode Toggle */}
            <div className="mode-toggle">
              <button
                className={`mode-btn ${mode === 'full_book' ? 'active' : ''}`}
                onClick={() => setMode('full_book')}
              >
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M4 19.5A2.5 2.5 0 016.5 17H20" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M6.5 2H20v20H6.5A2.5 2.5 0 014 19.5v-15A2.5 2.5 0 016.5 2z" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
                Full Course
              </button>
              <button
                className={`mode-btn ${mode === 'selected_text' ? 'active' : ''}`}
                onClick={() => setMode('selected_text')}
              >
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M11 4H4a2 2 0 00-2 2v14a2 2 0 002 2h14a2 2 0 002-2v-7" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M18.5 2.5a2.121 2.121 0 013 3L12 15l-4 1 1-4 9.5-9.5z" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
                Selected Text
              </button>
            </div>

            {/* Messages */}
            <div className="messages-container">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`message-wrapper ${message.sender}`}
                >
                  {message.sender === 'bot' && (
                    <div className="message-avatar">
                      <svg viewBox="0 0 24 24" fill="currentColor">
                        <path d="M12 2a2 2 0 012 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 017 7h1a1 1 0 011 1v3a1 1 0 01-1 1h-1v1a2 2 0 01-2 2H5a2 2 0 01-2-2v-1H2a1 1 0 01-1-1v-3a1 1 0 011-1h1a7 7 0 017-7h1V5.73c-.6-.34-1-.99-1-1.73a2 2 0 012-2z"/>
                      </svg>
                    </div>
                  )}
                  <div className={`message-bubble ${message.sender} ${message.isError ? 'error' : ''} ${message.isSystem ? 'system' : ''}`}>
                    <div className="message-content">{message.text}</div>
                    {message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        <details>
                          <summary>
                            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                              <path d="M13 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V9z" strokeLinecap="round" strokeLinejoin="round"/>
                              <path d="M13 2v7h7" strokeLinecap="round" strokeLinejoin="round"/>
                            </svg>
                            {message.sources.length} source{message.sources.length > 1 ? 's' : ''}
                          </summary>
                          <ul>
                            {message.sources.map((source, index) => (
                              <li key={index}>{source}</li>
                            ))}
                          </ul>
                        </details>
                      </div>
                    )}
                    {message.confidence && (
                      <div className="message-confidence">
                        <span className={`confidence-badge ${message.confidence > 0.7 ? 'high' : message.confidence > 0.4 ? 'medium' : 'low'}`}>
                          {Math.round(message.confidence * 100)}% confidence
                        </span>
                      </div>
                    )}
                    <div className="message-time">{formatTime(message.timestamp)}</div>
                  </div>
                </div>
              ))}

              {isLoading && (
                <div className="message-wrapper bot">
                  <div className="message-avatar">
                    <svg viewBox="0 0 24 24" fill="currentColor">
                      <path d="M12 2a2 2 0 012 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 017 7h1a1 1 0 011 1v3a1 1 0 01-1 1h-1v1a2 2 0 01-2 2H5a2 2 0 01-2-2v-1H2a1 1 0 01-1-1v-3a1 1 0 011-1h1a7 7 0 017-7h1V5.73c-.6-.34-1-.99-1-1.73a2 2 0 012-2z"/>
                    </svg>
                  </div>
                  <div className="message-bubble bot">
                    <div className="typing-animation">
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            {/* Input Area */}
            <div className="input-container">
              <div className="input-wrapper">
                <textarea
                  ref={inputRef}
                  value={inputText}
                  onChange={(e) => setInputText(e.target.value)}
                  onKeyDown={handleKeyDown}
                  placeholder={
                    isInputEnabled
                      ? "Type your question..."
                      : "Connecting to server..."
                  }
                  rows="1"
                  disabled={isLoading || !isInputEnabled}
                />
                <button
                  onClick={handleSendMessage}
                  disabled={!inputText.trim() || isLoading || !isInputEnabled}
                  className="send-btn"
                  title="Send message"
                >
                  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" strokeLinecap="round" strokeLinejoin="round"/>
                  </svg>
                </button>
              </div>
              <div className="input-hint">
                Press <kbd>Enter</kbd> to send {isDemoMode && <span className="demo-badge">Demo</span>}
              </div>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatButton;
