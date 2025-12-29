import React, { useState } from "react";
import { useAuth } from "../context/AuthContext";
import BackgroundSelector from "./BackgroundSelector";
import styles from "./AuthForms.module.css";

/**
 * SignupForm component for new user registration
 * Collects email, password, name, and background levels
 */
export default function SignupForm({ onSuccess, onSwitchToSignin }) {
  const { signUp } = useAuth();

  // Form state
  const [formData, setFormData] = useState({
    name: "",
    email: "",
    password: "",
    confirmPassword: "",
    softwareBackground: "beginner",
    hardwareBackground: "beginner",
  });

  // UI state
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [validationErrors, setValidationErrors] = useState({});

  // Handle input changes
  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));

    // Clear validation error when user starts typing
    if (validationErrors[name]) {
      setValidationErrors((prev) => ({ ...prev, [name]: null }));
    }
    // Clear general error when user starts typing
    if (error) {
      setError(null);
    }
  };

  // Handle background level changes
  const handleBackgroundChange = (field, value) => {
    setFormData((prev) => ({ ...prev, [field]: value }));
  };

  // Validate form
  const validateForm = () => {
    const errors = {};

    // Name validation
    if (!formData.name.trim()) {
      errors.name = "Name is required";
    }

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!formData.email) {
      errors.email = "Email is required";
    } else if (!emailRegex.test(formData.email)) {
      errors.email = "Please enter a valid email address";
    }

    // Password validation
    if (!formData.password) {
      errors.password = "Password is required";
    } else if (formData.password.length < 8) {
      errors.password = "Password must be at least 8 characters";
    }

    // Confirm password validation
    if (formData.password !== formData.confirmPassword) {
      errors.confirmPassword = "Passwords do not match";
    }

    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();
    setError(null);

    // Validate form
    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      console.log("Attempting sign up...");
      const result = await signUp.email({
        name: formData.name.trim(),
        email: formData.email.toLowerCase().trim(),
        password: formData.password,
        softwareBackground: formData.softwareBackground,
        hardwareBackground: formData.hardwareBackground,
        callbackURL: "/docs",
      });

      console.log("Sign up result:", result);

      if (result.error) {
        // Handle specific error codes with user-friendly messages
        const errorCode = result.error.code;
        const errorMessage = result.error.message;

        console.error("Sign up error:", errorCode, errorMessage);

        if (errorCode === "USER_ALREADY_EXISTS") {
          setError("An account with this email already exists. Please sign in instead.");
        } else if (errorCode === "RATE_LIMIT_EXCEEDED") {
          setError("Too many signup attempts. Please wait a moment and try again.");
        } else if (errorCode === "INTERNAL_ERROR" || errorMessage?.includes("fetch")) {
          setError("Unable to connect to the authentication server. Please check your connection and try again.");
        } else if (errorCode === "WEAK_PASSWORD") {
          setError("Password is too weak. Please use a stronger password.");
        } else {
          setError(result.error.message || "Failed to create account. Please try again.");
        }
      } else {
        // Success - call onSuccess callback
        console.log("Sign up successful!");
        if (onSuccess) {
          onSuccess(result.data);
        }
      }
    } catch (err) {
      console.error("Signup exception:", err);
      setError("An unexpected error occurred. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2 className={styles.formTitle}>Create Account</h2>
      <p className={styles.formSubtitle}>
        Join the Physical AI & Humanoid Robotics Course
      </p>

      {error && (
        <div className={styles.errorAlert} role="alert">
          {error}
        </div>
      )}

      {/* Name field */}
      <div className={styles.formGroup}>
        <label htmlFor="signup-name" className={styles.label}>
          Full Name
        </label>
        <input
          id="signup-name"
          type="text"
          name="name"
          value={formData.name}
          onChange={handleChange}
          placeholder="Enter your full name"
          className={`${styles.input} ${validationErrors.name ? styles.inputError : ""}`}
          disabled={isLoading}
          autoComplete="name"
        />
        {validationErrors.name && (
          <span className={styles.errorText}>{validationErrors.name}</span>
        )}
      </div>

      {/* Email field */}
      <div className={styles.formGroup}>
        <label htmlFor="signup-email" className={styles.label}>
          Email Address
        </label>
        <input
          id="signup-email"
          type="email"
          name="email"
          value={formData.email}
          onChange={handleChange}
          placeholder="your@email.com"
          className={`${styles.input} ${validationErrors.email ? styles.inputError : ""}`}
          disabled={isLoading}
          autoComplete="email"
        />
        {validationErrors.email && (
          <span className={styles.errorText}>{validationErrors.email}</span>
        )}
      </div>

      {/* Password field */}
      <div className={styles.formGroup}>
        <label htmlFor="signup-password" className={styles.label}>
          Password
        </label>
        <input
          id="signup-password"
          type="password"
          name="password"
          value={formData.password}
          onChange={handleChange}
          placeholder="Minimum 8 characters"
          className={`${styles.input} ${validationErrors.password ? styles.inputError : ""}`}
          disabled={isLoading}
          autoComplete="new-password"
        />
        {validationErrors.password && (
          <span className={styles.errorText}>{validationErrors.password}</span>
        )}
      </div>

      {/* Confirm Password field */}
      <div className={styles.formGroup}>
        <label htmlFor="signup-confirm-password" className={styles.label}>
          Confirm Password
        </label>
        <input
          id="signup-confirm-password"
          type="password"
          name="confirmPassword"
          value={formData.confirmPassword}
          onChange={handleChange}
          placeholder="Re-enter your password"
          className={`${styles.input} ${validationErrors.confirmPassword ? styles.inputError : ""}`}
          disabled={isLoading}
          autoComplete="new-password"
        />
        {validationErrors.confirmPassword && (
          <span className={styles.errorText}>{validationErrors.confirmPassword}</span>
        )}
      </div>

      {/* Background selectors */}
      <div className={styles.backgroundSection}>
        <h3 className={styles.sectionTitle}>Your Background (Optional)</h3>
        <p className={styles.sectionSubtitle}>
          Help us personalize content for your experience level
        </p>

        <BackgroundSelector
          label="Software/Programming"
          value={formData.softwareBackground}
          onChange={(value) => handleBackgroundChange("softwareBackground", value)}
          description="Your experience with programming, AI/ML, and ROS"
          disabled={isLoading}
        />

        <BackgroundSelector
          label="Hardware/Electronics"
          value={formData.hardwareBackground}
          onChange={(value) => handleBackgroundChange("hardwareBackground", value)}
          description="Your experience with electronics, robotics, and sensors"
          disabled={isLoading}
        />
      </div>

      {/* Submit button */}
      <button
        type="submit"
        className={styles.submitButton}
        disabled={isLoading}
      >
        {isLoading ? "Creating Account..." : "Create Account"}
      </button>

      {/* Switch to signin */}
      <p className={styles.switchText}>
        Already have an account?{" "}
        <button
          type="button"
          onClick={onSwitchToSignin}
          className={styles.linkButton}
          disabled={isLoading}
        >
          Sign In
        </button>
      </p>
    </form>
  );
}
