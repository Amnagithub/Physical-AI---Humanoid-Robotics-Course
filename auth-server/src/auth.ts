import { betterAuth } from "better-auth";
import { Pool } from "pg";
import "dotenv/config";

// Detect environment
const isDevelopment = process.env.NODE_ENV !== "production" || process.env.PORT === "3001";
const AUTH_PORT = parseInt(process.env.PORT || "3001", 10);

// Dynamic base URL based on environment
const BASE_URL = isDevelopment
  ? `http://localhost:${AUTH_PORT}`
  : process.env.BETTER_AUTH_URL || "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app";

console.log(`Auth config - Environment: ${isDevelopment ? "development" : "production"}, Base URL: ${BASE_URL}`);

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

// Better Auth configuration with custom user fields
export const auth = betterAuth({
  database: pool,

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true, // Auto sign-in after signup
  },

  // Custom user fields for personalization
  user: {
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: false,
        defaultValue: "beginner",
        input: true, // Allow setting during signup
      },
      hardwareBackground: {
        type: "string",
        required: false,
        defaultValue: "beginner",
        input: true,
      },
      preferredLanguage: {
        type: "string",
        required: false,
        defaultValue: "en",
        input: true,
      },
    },
  },

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24, // Update session every 24 hours
  },

  // Trusted origins for CORS - dynamically configured
  trustedOrigins: isDevelopment
    ? [
        "http://localhost:3000", // Docusaurus dev
        "http://localhost:3001", // Auth server dev
        "http://localhost:8000", // FastAPI backend
        "http://127.0.0.1:3000",
        "http://127.0.0.1:8000",
      ]
    : [
        process.env.BETTER_AUTH_URL || "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app",
      ],

  // Secret key for signing
  secret: process.env.BETTER_AUTH_SECRET,

  // Base URL
  baseURL: BASE_URL,
});

// Export auth type for type inference
export type Auth = typeof auth;
