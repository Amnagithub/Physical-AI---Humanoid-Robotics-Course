import React, { createContext, useContext, useMemo } from "react";
import { authClient } from "../lib/auth-client";

// Create the auth context
const AuthContext = createContext(null);

/**
 * AuthProvider component that wraps the app and provides auth state
 */
export function AuthProvider({ children }) {
  // Use Better Auth's useSession hook
  const { data: sessionData, isPending: isLoading, error } = authClient.useSession();

  // Memoize the context value to prevent unnecessary re-renders
  const value = useMemo(
    () => ({
      // User and session data
      user: sessionData?.user || null,
      session: sessionData?.session || null,

      // Auth state
      isLoading,
      isAuthenticated: !!sessionData?.user,
      error,

      // Auth methods
      signIn: authClient.signIn,
      signUp: authClient.signUp,
      signOut: authClient.signOut,
      updateUser: authClient.updateUser,
      getSession: authClient.getSession,
    }),
    [sessionData, isLoading, error]
  );

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * Hook to access auth context
 * @returns Auth context value with user, session, and auth methods
 */
export function useAuth() {
  const context = useContext(AuthContext);

  if (context === null) {
    throw new Error("useAuth must be used within an AuthProvider");
  }

  return context;
}

/**
 * Hook to check if user is authenticated
 * @returns Boolean indicating authentication status
 */
export function useIsAuthenticated() {
  const { isAuthenticated, isLoading } = useAuth();
  return { isAuthenticated, isLoading };
}

/**
 * Hook to get current user
 * @returns Current user object or null
 */
export function useUser() {
  const { user, isLoading } = useAuth();
  return { user, isLoading };
}

export default AuthContext;
