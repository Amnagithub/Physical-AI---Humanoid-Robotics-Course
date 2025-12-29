import { betterAuth } from "better-auth";
import { Pool } from "pg";
import "dotenv/config";

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

  // Trusted origins for CORS
  trustedOrigins: [
    process.env.BETTER_AUTH_URL || "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app",
    "http://localhost:3000", // Docusaurus dev
    "http://localhost:3001", // Auth server dev
  ],

  // Secret key for signing
  secret: process.env.BETTER_AUTH_SECRET,

  // Base URL
  baseURL: process.env.BETTER_AUTH_URL,
});

// Export auth type for type inference
export type Auth = typeof auth;
