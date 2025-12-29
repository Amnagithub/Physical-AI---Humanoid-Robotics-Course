/**
 * Type definitions for the auth server
 */

// Background level enum
export type BackgroundLevel = "beginner" | "intermediate" | "advanced";

// User interface with custom fields
export interface User {
  id: string;
  email: string;
  emailVerified: boolean;
  name: string;
  image: string | null;
  createdAt: Date;
  updatedAt: Date;
  // Custom fields for personalization
  softwareBackground: BackgroundLevel;
  hardwareBackground: BackgroundLevel;
  preferredLanguage: string;
}

// Session interface
export interface Session {
  id: string;
  userId: string;
  token: string;
  expiresAt: Date;
  ipAddress: string | null;
  userAgent: string | null;
  createdAt: Date;
  updatedAt: Date;
}

// Combined user session
export interface UserSession {
  user: User;
  session: Session;
}

// Signup request
export interface SignUpRequest {
  name: string;
  email: string;
  password: string;
  image?: string;
  softwareBackground?: BackgroundLevel;
  hardwareBackground?: BackgroundLevel;
  callbackURL?: string;
}

// Signin request
export interface SignInRequest {
  email: string;
  password: string;
  rememberMe?: boolean;
  callbackURL?: string;
}

// Update user request
export interface UpdateUserRequest {
  name?: string;
  image?: string | null;
  softwareBackground?: BackgroundLevel;
  hardwareBackground?: BackgroundLevel;
  preferredLanguage?: string;
}

// Auth response
export interface AuthResponse {
  user: User;
  session: Session;
}

// Error response
export interface ErrorResponse {
  error: {
    message: string;
    code: string;
    status: number;
  };
}

// Validation helpers
export const BACKGROUND_LEVELS: BackgroundLevel[] = ["beginner", "intermediate", "advanced"];

export function isValidBackgroundLevel(level: string): level is BackgroundLevel {
  return BACKGROUND_LEVELS.includes(level as BackgroundLevel);
}

export function isValidLanguageCode(code: string): boolean {
  return /^[a-z]{2}$/.test(code);
}
