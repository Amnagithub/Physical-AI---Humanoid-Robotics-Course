import React from "react";
import styles from "./AuthForms.module.css";

/**
 * BackgroundSelector component for selecting experience level
 * Used for softwareBackground and hardwareBackground fields
 */
export default function BackgroundSelector({
  label,
  value,
  onChange,
  description,
  disabled = false,
}) {
  const levels = [
    {
      value: "beginner",
      label: "Beginner",
      description: "New to this area, learning the basics",
    },
    {
      value: "intermediate",
      label: "Intermediate",
      description: "Comfortable with fundamentals, some practical experience",
    },
    {
      value: "advanced",
      label: "Advanced",
      description: "Deep knowledge and significant hands-on experience",
    },
  ];

  return (
    <div className={styles.backgroundSelector}>
      <label className={styles.selectorLabel}>
        {label}
        {description && (
          <span className={styles.selectorDescription}>{description}</span>
        )}
      </label>

      <div className={styles.levelOptions}>
        {levels.map((level) => (
          <button
            key={level.value}
            type="button"
            onClick={() => onChange(level.value)}
            className={`${styles.levelOption} ${
              value === level.value ? styles.levelOptionSelected : ""
            }`}
            disabled={disabled}
            title={level.description}
          >
            <span className={styles.levelLabel}>{level.label}</span>
            <span className={styles.levelIndicator}>
              {value === level.value && "✓"}
            </span>
          </button>
        ))}
      </div>
    </div>
  );
}
