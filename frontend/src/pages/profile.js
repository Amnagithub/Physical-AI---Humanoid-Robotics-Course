import React, { useState, useEffect } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../context/AuthContext";
import BackgroundSelector from "../components/BackgroundSelector";
import styles from "./profile.module.css";

// API base URL for cache invalidation
const API_BASE_URL =
  typeof window !== "undefined" && window.location.hostname !== "localhost"
    ? "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app"
    : "http://localhost:8000";

export default function ProfilePage() {
  const { user, isAuthenticated, isLoading, updateUser } = useAuth();

  // Form state
  const [formData, setFormData] = useState({
    name: "",
    softwareBackground: "beginner",
    hardwareBackground: "beginner",
  });

  // UI state
  const [isSaving, setIsSaving] = useState(false);
  const [saveStatus, setSaveStatus] = useState(null); // 'success' | 'error' | null
  const [statusMessage, setStatusMessage] = useState("");

  // Initialize form with user data
  useEffect(() => {
    if (user) {
      setFormData({
        name: user.name || "",
        softwareBackground: user.softwareBackground || "beginner",
        hardwareBackground: user.hardwareBackground || "beginner",
      });
    }
  }, [user]);

  // Redirect to home if not authenticated
  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      window.location.href = "/";
    }
  }, [isLoading, isAuthenticated]);

  // Handle form changes
  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    setSaveStatus(null);
  };

  // Handle background level changes
  const handleBackgroundChange = (field, value) => {
    setFormData((prev) => ({ ...prev, [field]: value }));
    setSaveStatus(null);
  };

  // Invalidate user's personalization cache
  const invalidateCache = async () => {
    if (!user?.id) return;

    try {
      await fetch(`${API_BASE_URL}/api/v1/content/cache`, {
        method: "DELETE",
        headers: {
          "X-User-Id": user.id,
        },
      });
    } catch (err) {
      console.warn("Failed to invalidate cache:", err);
    }
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsSaving(true);
    setSaveStatus(null);

    try {
      // Update user profile via Better Auth
      const result = await updateUser({
        name: formData.name,
        softwareBackground: formData.softwareBackground,
        hardwareBackground: formData.hardwareBackground,
      });

      if (result?.error) {
        throw new Error(result.error.message || "Failed to update profile");
      }

      // Invalidate personalization cache since background changed
      await invalidateCache();

      setSaveStatus("success");
      setStatusMessage("Profile updated successfully! Your content will be re-personalized with your new settings.");
    } catch (err) {
      console.error("Profile update error:", err);
      setSaveStatus("error");
      setStatusMessage(err.message || "Failed to update profile. Please try again.");
    } finally {
      setIsSaving(false);
    }
  };

  // Loading state
  if (isLoading) {
    return (
      <Layout title="Profile">
        <main className={styles.container}>
          <div className={styles.loading}>Loading...</div>
        </main>
      </Layout>
    );
  }

  // Not authenticated
  if (!isAuthenticated) {
    return null; // Will redirect
  }

  return (
    <Layout title="My Profile">
      <main className={styles.container}>
        <div className={styles.profileCard}>
          <h1 className={styles.title}>My Profile</h1>
          <p className={styles.subtitle}>
            Update your profile to personalize your learning experience
          </p>

          <form onSubmit={handleSubmit} className={styles.form}>
            {/* Status message */}
            {saveStatus && (
              <div
                className={`${styles.statusMessage} ${
                  saveStatus === "success" ? styles.success : styles.error
                }`}
                role="alert"
              >
                {statusMessage}
              </div>
            )}

            {/* Email (read-only) */}
            <div className={styles.formGroup}>
              <label className={styles.label}>Email</label>
              <input
                type="email"
                value={user?.email || ""}
                className={styles.input}
                disabled
                readOnly
              />
              <span className={styles.hint}>Email cannot be changed</span>
            </div>

            {/* Name */}
            <div className={styles.formGroup}>
              <label htmlFor="name" className={styles.label}>
                Display Name
              </label>
              <input
                id="name"
                type="text"
                name="name"
                value={formData.name}
                onChange={handleChange}
                className={styles.input}
                placeholder="Your name"
                disabled={isSaving}
              />
            </div>

            {/* Software Background */}
            <div className={styles.formGroup}>
              <label className={styles.label}>Software Background</label>
              <p className={styles.fieldDescription}>
                Your programming experience level affects how technical concepts are explained
              </p>
              <BackgroundSelector
                label=""
                value={formData.softwareBackground}
                onChange={(value) => handleBackgroundChange("softwareBackground", value)}
                disabled={isSaving}
              />
            </div>

            {/* Hardware Background */}
            <div className={styles.formGroup}>
              <label className={styles.label}>Hardware Background</label>
              <p className={styles.fieldDescription}>
                Your robotics/electronics experience affects hardware-related explanations
              </p>
              <BackgroundSelector
                label=""
                value={formData.hardwareBackground}
                onChange={(value) => handleBackgroundChange("hardwareBackground", value)}
                disabled={isSaving}
              />
            </div>

            {/* Submit button */}
            <button
              type="submit"
              className={styles.submitButton}
              disabled={isSaving}
            >
              {isSaving ? "Saving..." : "Save Changes"}
            </button>

            {/* Info box */}
            <div className={styles.infoBox}>
              <strong>Note:</strong> Changing your background levels will clear your
              personalized content cache. The next time you personalize a chapter,
              it will use your updated profile settings.
            </div>
          </form>
        </div>
      </main>
    </Layout>
  );
}
