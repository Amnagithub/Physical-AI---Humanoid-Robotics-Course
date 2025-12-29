import React, { useState, useEffect, useCallback } from "react";
import SignupForm from "./SignupForm";
import SigninForm from "./SigninForm";
import styles from "./AuthModal.module.css";

/**
 * AuthModal component - Modal dialog for authentication
 * Contains tabs for Sign In and Sign Up
 */
export default function AuthModal({ isOpen, onClose, defaultTab = "signin" }) {
  const [activeTab, setActiveTab] = useState(defaultTab);

  // Update active tab when defaultTab prop changes
  useEffect(() => {
    if (isOpen) {
      setActiveTab(defaultTab);
    }
  }, [isOpen, defaultTab]);

  // Handle escape key to close modal
  const handleKeyDown = useCallback(
    (e) => {
      if (e.key === "Escape") {
        onClose();
      }
    },
    [onClose]
  );

  // Add/remove event listener
  useEffect(() => {
    if (isOpen) {
      document.addEventListener("keydown", handleKeyDown);
      // Prevent body scroll when modal is open
      document.body.style.overflow = "hidden";
    }

    return () => {
      document.removeEventListener("keydown", handleKeyDown);
      document.body.style.overflow = "";
    };
  }, [isOpen, handleKeyDown]);

  // Handle successful authentication
  const handleAuthSuccess = (data) => {
    onClose();
    // Redirect to docs after short delay for cookie to be set
    setTimeout(() => {
      window.location.href = "/docs";
    }, 100);
  };

  // Don't render if not open
  if (!isOpen) {
    return null;
  }

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div
        className={styles.modalContent}
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-labelledby="auth-modal-title"
      >
        {/* Close button */}
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close modal"
        >
          &times;
        </button>

        {/* Tab navigation */}
        <div className={styles.tabNav}>
          <button
            className={`${styles.tab} ${activeTab === "signin" ? styles.tabActive : ""}`}
            onClick={() => setActiveTab("signin")}
          >
            Sign In
          </button>
          <button
            className={`${styles.tab} ${activeTab === "signup" ? styles.tabActive : ""}`}
            onClick={() => setActiveTab("signup")}
          >
            Sign Up
          </button>
        </div>

        {/* Tab content */}
        <div className={styles.tabContent}>
          {activeTab === "signin" ? (
            <SigninForm
              onSuccess={handleAuthSuccess}
              onSwitchToSignup={() => setActiveTab("signup")}
            />
          ) : (
            <SignupForm
              onSuccess={handleAuthSuccess}
              onSwitchToSignin={() => setActiveTab("signin")}
            />
          )}
        </div>
      </div>
    </div>
  );
}
