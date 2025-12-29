import { createAuthClient } from "better-auth/react";
import { inferAdditionalFields } from "better-auth/client/plugins";

// Auth server base URL - uses environment variable or defaults
const AUTH_BASE_URL =
  typeof window !== "undefined"
    ? window.location.hostname === "localhost"
      ? "http://localhost:3001"
      : "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app"
    : "http://localhost:3001";

// Create the auth client with custom fields inference
export const authClient = createAuthClient({
  baseURL: AUTH_BASE_URL,
  plugins: [
    inferAdditionalFields({
      user: {
        softwareBackground: {
          type: "string",
          required: false,
        },
        hardwareBackground: {
          type: "string",
          required: false,
        },
        preferredLanguage: {
          type: "string",
          required: false,
        },
      },
    }),
  ],
});

// Export commonly used functions and hooks
export const {
  useSession,
  signIn,
  signUp,
  signOut,
  getSession,
  updateUser,
} = authClient;

// Type exports for components
export type BackgroundLevel = "beginner" | "intermediate" | "advanced";

export interface UserProfile {
  id: string;
  email: string;
  name: string;
  image: string | null;
  softwareBackground: BackgroundLevel;
  hardwareBackground: BackgroundLevel;
  preferredLanguage: string;
}

// Helper to check if user is authenticated
export function isAuthenticated(session: unknown): boolean {
  return session !== null && typeof session === "object" && "user" in session;
}
