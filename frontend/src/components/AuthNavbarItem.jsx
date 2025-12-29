import React, { useState } from "react";
import UserProfileButton from "./UserProfileButton";
import AuthModal from "./AuthModal";

/**
 * AuthNavbarItem - Custom navbar item for authentication
 * Shows UserProfileButton which displays sign in or user menu
 * Opens AuthModal when sign in is clicked
 */
export default function AuthNavbarItem() {
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleSignInClick = () => {
    setIsModalOpen(true);
  };

  const handleModalClose = () => {
    setIsModalOpen(false);
  };

  const handleAuthSuccess = () => {
    setIsModalOpen(false);
    // Reload the page to refresh auth state
    window.location.reload();
  };

  return (
    <>
      <UserProfileButton onSignInClick={handleSignInClick} />
      <AuthModal
        isOpen={isModalOpen}
        onClose={handleModalClose}
        onSuccess={handleAuthSuccess}
      />
    </>
  );
}
