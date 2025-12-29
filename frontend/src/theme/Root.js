import React from 'react';
import { AuthProvider } from '../context/AuthContext';
import FloatingChatButton from '../components/FloatingChatButton';

// API base URL - uses environment variable or defaults
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://physical-ai-humanoid-robotics-cours-ashen.vercel.app'
  : 'http://localhost:8000';

// This wrapper component adds auth provider and chatbot to all pages
export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <FloatingChatButton apiBaseUrl={API_BASE_URL} />
    </AuthProvider>
  );
}
