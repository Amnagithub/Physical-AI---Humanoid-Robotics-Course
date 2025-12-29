import React, { useState, useCallback } from "react";
import { useAuth } from "../context/AuthContext";
import styles from "./ChapterPersonalization.module.css";

// API base URL
const API_BASE_URL =
  typeof window !== "undefined" && window.location.hostname !== "localhost"
    ? "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app"
    : "http://localhost:8000";

/**
 * ChapterPersonalization component
 * Adds personalization and translation controls to chapter content
 */
export default function ChapterPersonalization({ originalContent, chapterPath }) {
  const { user, isAuthenticated } = useAuth();

  // Content state
  const [activeContent, setActiveContent] = useState(null); // null = showing original
  const [contentLanguage, setContentLanguage] = useState("en");

  // UI state
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState(null);

  // Get current chapter path from window location
  const getCurrentPath = useCallback(() => {
    if (chapterPath) return chapterPath;
    if (typeof window !== "undefined") {
      return window.location.pathname;
    }
    return "/docs/unknown";
  }, [chapterPath]);

  // Personalize content
  const handlePersonalize = async () => {
    if (!isAuthenticated || !user) return;

    setIsPersonalizing(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/v1/content/personalize`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-User-Id": user.id,
        },
        body: JSON.stringify({
          chapter_path: getCurrentPath(),
          original_content: originalContent,
          user_profile: {
            software_background: user.softwareBackground || "beginner",
            hardware_background: user.hardwareBackground || "beginner",
          },
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || "Failed to personalize content");
      }

      const data = await response.json();
      setActiveContent(data.content);
      setContentLanguage("en");
    } catch (err) {
      console.error("Personalization error:", err);
      setError(err.message || "Failed to personalize content. Please try again.");
    } finally {
      setIsPersonalizing(false);
    }
  };

  // Translate content
  const handleTranslate = async () => {
    if (!isAuthenticated || !user) return;

    setIsTranslating(true);
    setError(null);

    try {
      // Translate the current active content (personalized or original)
      const contentToTranslate = activeContent || originalContent;

      const response = await fetch(`${API_BASE_URL}/api/v1/content/translate`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-User-Id": user.id,
        },
        body: JSON.stringify({
          chapter_path: getCurrentPath(),
          content: contentToTranslate,
          target_language: "ur",
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || "Failed to translate content");
      }

      const data = await response.json();
      setActiveContent(data.content);
      setContentLanguage("ur");
    } catch (err) {
      console.error("Translation error:", err);
      setError(err.message || "Failed to translate content. Please try again.");
    } finally {
      setIsTranslating(false);
    }
  };

  // Show original content
  const handleShowOriginal = () => {
    setActiveContent(null);
    setContentLanguage("en");
    setError(null);
  };

  // Check if showing modified content
  const isShowingModified = activeContent !== null;
  const isShowingUrdu = contentLanguage === "ur";

  return (
    <div className={styles.container}>
      {/* Control bar */}
      <div className={styles.controlBar}>
        <div className={styles.buttonGroup}>
          {/* Personalize button */}
          <button
            className={`${styles.button} ${styles.personalizeButton}`}
            onClick={handlePersonalize}
            disabled={!isAuthenticated || isPersonalizing || isTranslating}
            title={
              !isAuthenticated
                ? "Sign in to personalize content"
                : "Adapt content to your background level"
            }
          >
            {isPersonalizing ? (
              <>
                <span className={styles.spinner} />
                Personalizing...
              </>
            ) : (
              <>
                <span className={styles.icon}>&#9881;</span>
                Personalize
              </>
            )}
          </button>

          {/* Translate button */}
          <button
            className={`${styles.button} ${styles.translateButton}`}
            onClick={handleTranslate}
            disabled={!isAuthenticated || isPersonalizing || isTranslating || isShowingUrdu}
            title={
              !isAuthenticated
                ? "Sign in to translate content"
                : isShowingUrdu
                ? "Already showing Urdu"
                : "Translate to Urdu"
            }
          >
            {isTranslating ? (
              <>
                <span className={styles.spinner} />
                Translating...
              </>
            ) : (
              <>
                <span className={styles.icon}>&#127760;</span>
                {isShowingUrdu ? "Urdu" : "Translate to Urdu"}
              </>
            )}
          </button>

          {/* Show English button (when showing Urdu) */}
          {isShowingUrdu && (
            <button
              className={`${styles.button} ${styles.englishButton}`}
              onClick={() => {
                setContentLanguage("en");
                // Keep personalized content if it exists, otherwise show original
                if (!activeContent) {
                  setActiveContent(null);
                }
              }}
              disabled={isPersonalizing || isTranslating}
            >
              <span className={styles.icon}>EN</span>
              English
            </button>
          )}

          {/* Show Original button */}
          {isShowingModified && (
            <button
              className={`${styles.button} ${styles.originalButton}`}
              onClick={handleShowOriginal}
              disabled={isPersonalizing || isTranslating}
            >
              <span className={styles.icon}>&#8634;</span>
              Show Original
            </button>
          )}
        </div>

        {/* Status indicator */}
        {isShowingModified && (
          <div className={styles.statusBadge}>
            {isShowingUrdu ? "Urdu Translation" : "Personalized Content"}
          </div>
        )}

        {/* Auth hint for non-authenticated users */}
        {!isAuthenticated && (
          <div className={styles.authHint}>
            Sign in to personalize and translate content
          </div>
        )}
      </div>

      {/* Error message */}
      {error && (
        <div className={styles.errorMessage} role="alert">
          {error}
          <button
            className={styles.dismissError}
            onClick={() => setError(null)}
            aria-label="Dismiss error"
          >
            &times;
          </button>
        </div>
      )}

      {/* Content display - render modified content or let original pass through */}
      {isShowingModified && (
        <div
          className={`${styles.modifiedContent} ${isShowingUrdu ? styles.urduContent : ""}`}
          dir={isShowingUrdu ? "rtl" : "ltr"}
          dangerouslySetInnerHTML={{ __html: activeContent }}
        />
      )}
    </div>
  );
}
