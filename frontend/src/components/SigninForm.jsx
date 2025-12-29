import React, { useState } from "react";
import { useAuth } from "../context/AuthContext";
import styles from "./AuthForms.module.css";

/**
 * SigninForm component for returning user authentication
 * Collects email, password, and rememberMe option
 */
export default function SigninForm({ onSuccess, onSwitchToSignup }) {
  const { signIn } = useAuth();

  // Form state
  const [formData, setFormData] = useState({
    email: "",
    password: "",
    rememberMe: true,
  });

  // UI state
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [validationErrors, setValidationErrors] = useState({});

  // Handle input changes
  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: type === "checkbox" ? checked : value,
    }));

    // Clear validation error when user starts typing
    if (validationErrors[name]) {
      setValidationErrors((prev) => ({ ...prev, [name]: null }));
    }
    // Clear general error when user starts typing
    if (error) {
      setError(null);
    }
  };

  // Validate form
  const validateForm = () => {
    const errors = {};

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
      const result = await signIn.email({
        email: formData.email.toLowerCase().trim(),
        password: formData.password,
        rememberMe: formData.rememberMe,
        callbackURL: "/docs",
      });

      if (result.error) {
        // Handle specific error codes
        if (result.error.code === "INVALID_CREDENTIALS" ||
            result.error.code === "USER_NOT_FOUND") {
          setError("Invalid email or password. Please try again.");
        } else {
          setError(result.error.message || "Failed to sign in. Please try again.");
        }
      } else {
        // Success - call onSuccess callback
        if (onSuccess) {
          onSuccess(result.data);
        }
      }
    } catch (err) {
      console.error("Signin error:", err);
      setError("An unexpected error occurred. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2 className={styles.formTitle}>Welcome Back</h2>
      <p className={styles.formSubtitle}>
        Sign in to continue your learning journey
      </p>

      {error && (
        <div className={styles.errorAlert} role="alert">
          {error}
        </div>
      )}

      {/* Email field */}
      <div className={styles.formGroup}>
        <label htmlFor="signin-email" className={styles.label}>
          Email Address
        </label>
        <input
          id="signin-email"
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
        <label htmlFor="signin-password" className={styles.label}>
          Password
        </label>
        <input
          id="signin-password"
          type="password"
          name="password"
          value={formData.password}
          onChange={handleChange}
          placeholder="Enter your password"
          className={`${styles.input} ${validationErrors.password ? styles.inputError : ""}`}
          disabled={isLoading}
          autoComplete="current-password"
        />
        {validationErrors.password && (
          <span className={styles.errorText}>{validationErrors.password}</span>
        )}
      </div>

      {/* Remember me checkbox */}
      <div className={styles.checkboxGroup}>
        <input
          id="signin-remember"
          type="checkbox"
          name="rememberMe"
          checked={formData.rememberMe}
          onChange={handleChange}
          className={styles.checkbox}
          disabled={isLoading}
        />
        <label htmlFor="signin-remember" className={styles.checkboxLabel}>
          Remember me for 7 days
        </label>
      </div>

      {/* Submit button */}
      <button
        type="submit"
        className={styles.submitButton}
        disabled={isLoading}
      >
        {isLoading ? "Signing In..." : "Sign In"}
      </button>

      {/* Switch to signup */}
      <p className={styles.switchText}>
        Don't have an account?{" "}
        <button
          type="button"
          onClick={onSwitchToSignup}
          className={styles.linkButton}
          disabled={isLoading}
        >
          Create Account
        </button>
      </p>
    </form>
  );
}
