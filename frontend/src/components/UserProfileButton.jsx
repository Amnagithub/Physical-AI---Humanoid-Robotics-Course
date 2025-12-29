import React, { useState, useRef, useEffect } from "react";
import { useAuth } from "../context/AuthContext";
import styles from "./UserProfileButton.module.css";

/**
 * UserProfileButton component for navbar
 * Shows user name and dropdown menu when logged in
 * Shows Sign In button when logged out
 */
export default function UserProfileButton({ onSignInClick }) {
  const { user, isLoading, isAuthenticated, signOut } = useAuth();
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const dropdownRef = useRef(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsDropdownOpen(false);
      }
    };

    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, []);

  // Handle sign out
  const handleSignOut = async () => {
    setIsDropdownOpen(false);
    try {
      await signOut();
      // Redirect to home after sign out
      window.location.href = "/";
    } catch (error) {
      console.error("Sign out error:", error);
    }
  };

  // Handle profile click
  const handleProfileClick = () => {
    setIsDropdownOpen(false);
    window.location.href = "/profile";
  };

  // Loading state
  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loadingIndicator} />
      </div>
    );
  }

  // Not authenticated - show sign in button
  if (!isAuthenticated) {
    return (
      <button className={styles.signInButton} onClick={onSignInClick}>
        Sign In
      </button>
    );
  }

  // Authenticated - show user button with dropdown
  const initials = user?.name
    ? user.name
        .split(" ")
        .map((n) => n[0])
        .join("")
        .toUpperCase()
        .slice(0, 2)
    : "U";

  return (
    <div className={styles.container} ref={dropdownRef}>
      <button
        className={styles.userButton}
        onClick={() => setIsDropdownOpen(!isDropdownOpen)}
        aria-expanded={isDropdownOpen}
        aria-haspopup="true"
      >
        <span className={styles.avatar}>{initials}</span>
        <span className={styles.userName}>{user?.name || "User"}</span>
        <span className={styles.chevron}>{isDropdownOpen ? "▲" : "▼"}</span>
      </button>

      {isDropdownOpen && (
        <div className={styles.dropdown}>
          <div className={styles.dropdownHeader}>
            <span className={styles.dropdownEmail}>{user?.email}</span>
            <span className={styles.dropdownLevel}>
              SW: {user?.softwareBackground || "beginner"} |{" "}
              HW: {user?.hardwareBackground || "beginner"}
            </span>
          </div>

          <div className={styles.dropdownDivider} />

          <button className={styles.dropdownItem} onClick={handleProfileClick}>
            <span className={styles.dropdownIcon}>👤</span>
            My Profile
          </button>

          <div className={styles.dropdownDivider} />

          <button
            className={`${styles.dropdownItem} ${styles.dropdownItemDanger}`}
            onClick={handleSignOut}
          >
            <span className={styles.dropdownIcon}>🚪</span>
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
