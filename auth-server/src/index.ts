import { Hono } from "hono";
import { cors } from "hono/cors";
import { logger } from "hono/logger";
import { serve } from "@hono/node-server";
import { auth } from "./auth";
import "dotenv/config";

const app = new Hono();

// Detect environment
const isDevelopment = process.env.NODE_ENV !== "production" || process.env.PORT === "3001";
const AUTH_PORT = parseInt(process.env.PORT || "3001", 10);

// Dynamic base URL based on environment
const BASE_URL = isDevelopment
  ? `http://localhost:${AUTH_PORT}`
  : process.env.BETTER_AUTH_URL || "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app";

console.log(`Auth server environment: ${isDevelopment ? "development" : "production"}`);
console.log(`Auth server base URL: ${BASE_URL}`);

// Simple in-memory rate limiter
const rateLimitStore = new Map<string, { count: number; resetTime: number }>();

function rateLimit(
  windowMs: number,
  maxRequests: number,
  keyGenerator: (ip: string, path: string) => string
) {
  return async (c: any, next: () => Promise<void>) => {
    const ip = c.req.header("x-forwarded-for") || c.req.header("x-real-ip") || "unknown";
    const path = c.req.path;
    const key = keyGenerator(ip, path);
    const now = Date.now();

    const record = rateLimitStore.get(key);

    if (!record || now > record.resetTime) {
      rateLimitStore.set(key, { count: 1, resetTime: now + windowMs });
      return next();
    }

    if (record.count >= maxRequests) {
      console.warn(`Rate limit exceeded for ${key}`);
      return c.json(
        {
          error: {
            message: "Too many requests. Please try again later.",
            code: "RATE_LIMIT_EXCEEDED",
            status: 429,
          },
        },
        429
      );
    }

    record.count++;
    return next();
  };
}

// Cleanup old rate limit entries every 5 minutes
setInterval(() => {
  const now = Date.now();
  for (const [key, value] of rateLimitStore.entries()) {
    if (now > value.resetTime) {
      rateLimitStore.delete(key);
    }
  }
}, 5 * 60 * 1000);

// Middleware: Request logging
app.use("*", logger());

// CORS origins - dynamically configured based on environment
const corsOrigins = isDevelopment
  ? [
      "http://localhost:3000", // Docusaurus dev server
      "http://localhost:8000", // FastAPI backend
      "http://localhost:3001", // Auth server itself
      "http://127.0.0.1:3000",
      "http://127.0.0.1:8000",
    ]
  : [
      "https://physical-ai-humanoid-robotics-cours-ashen.vercel.app",
    ];

console.log(`CORS origins configured: ${JSON.stringify(corsOrigins)}`);

// Middleware: CORS configuration
app.use(
  "*",
  cors({
    origin: corsOrigins,
    credentials: true,
    allowMethods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowHeaders: ["Content-Type", "Authorization", "Cookie"],
    exposeHeaders: ["Set-Cookie"],
  })
);

// Health check endpoint
app.get("/health", (c) => {
  console.log("Health check requested");
  return c.json({
    status: "ok",
    timestamp: new Date().toISOString(),
    service: "auth-server",
    environment: isDevelopment ? "development" : "production",
    baseUrl: BASE_URL,
  });
});

// Rate limiting for auth endpoints
// Signup: 5 requests per minute per IP
app.use(
  "/api/auth/sign-up/*",
  rateLimit(60 * 1000, 5, (ip) => `signup:${ip}`)
);

// Signin: 10 requests per minute per IP
app.use(
  "/api/auth/sign-in/*",
  rateLimit(60 * 1000, 10, (ip) => `signin:${ip}`)
);

// Better Auth handler - handles all /api/auth/* routes
app.on(["GET", "POST"], "/api/auth/*", (c) => {
  console.log(`Auth request: ${c.req.method} ${c.req.url}`);
  return auth.handler(c.req.raw);
});

// 404 handler for unmatched routes
app.notFound((c) => {
  console.log(`404 - Not Found: ${c.req.path}`);
  return c.json(
    {
      error: {
        message: "Not Found",
        code: "NOT_FOUND",
        status: 404,
      },
    },
    404
  );
});

// Global error handler
app.onError((err, c) => {
  console.error("Auth server error:", err);
  return c.json(
    {
      error: {
        message: err.message || "Internal Server Error",
        code: "INTERNAL_ERROR",
        status: 500,
      },
    },
    500
  );
});

// Start server
console.log(`Auth server starting on port ${AUTH_PORT}...`);

serve(
  {
    fetch: app.fetch,
    port: AUTH_PORT,
  },
  (info) => {
    console.log(`Auth server running on http://localhost:${info.port}`);
    console.log(`Health check: http://localhost:${info.port}/health`);
    console.log(`Auth endpoints: http://localhost:${info.port}/api/auth/*`);
  }
);

export default app;
