import React from 'react';
import FloatingChatButton from '../components/FloatingChatButton';

// This wrapper component adds the chatbot to all pages
export default function Root({ children }) {
  return (
    <>
      {children}
      <FloatingChatButton apiBaseUrl="http://localhost:8000" />
    </>
  );
}
