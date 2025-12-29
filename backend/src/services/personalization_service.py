"""
Personalization service for adapting and translating chapter content
Feature: 002-user-auth-personalization

Uses Cohere LLM for content transformation while preserving technical accuracy.
"""

import cohere
import hashlib
from typing import Optional, Dict, Any
from datetime import datetime, timedelta

from ..config import settings
from ..schemas.personalization import (
    BackgroundLevel,
    ContentType,
    UserProfile,
    ContentResponse,
)


class PersonalizationService:
    """Service for personalizing and translating course content using Cohere LLM"""

    def __init__(self):
        self.cohere_client = cohere.ClientV2(api_key=settings.COHERE_API_KEY)
        self.model = "command-a-03-2025"
        self.cache_expiry_days = 7

    def _compute_content_hash(self, content: str) -> str:
        """Compute MD5 hash of content for cache validation"""
        return hashlib.md5(content.encode()).hexdigest()

    def _get_background_description(self, level: BackgroundLevel, domain: str) -> str:
        """Get human-readable description of background level"""
        descriptions = {
            BackgroundLevel.BEGINNER: {
                "software": "new to programming with minimal coding experience",
                "hardware": "unfamiliar with electronics and robotics hardware"
            },
            BackgroundLevel.INTERMEDIATE: {
                "software": "comfortable with programming basics and familiar with at least one language",
                "hardware": "has some experience with electronics, sensors, or microcontrollers"
            },
            BackgroundLevel.ADVANCED: {
                "software": "experienced developer with strong programming skills",
                "hardware": "experienced with robotics hardware, embedded systems, and electronics"
            }
        }
        return descriptions.get(level, descriptions[BackgroundLevel.BEGINNER]).get(domain, "")

    def _build_personalization_prompt(
        self,
        content: str,
        user_profile: UserProfile
    ) -> str:
        """
        Build the prompt for personalizing content based on user background.

        The prompt instructs the LLM to adapt explanations, examples, and
        complexity while preserving technical accuracy and code blocks.
        """
        sw_desc = self._get_background_description(user_profile.software_background, "software")
        hw_desc = self._get_background_description(user_profile.hardware_background, "hardware")

        return f"""You are an expert robotics and ROS 2 instructor. Your task is to personalize the following technical content for a specific learner.

LEARNER PROFILE:
- Software background: {user_profile.software_background.value} - {sw_desc}
- Hardware background: {user_profile.hardware_background.value} - {hw_desc}

PERSONALIZATION GUIDELINES:

For BEGINNER software level:
- Add brief explanations of programming concepts when they appear
- Use more analogies and real-world comparisons
- Simplify code explanations with step-by-step breakdowns

For INTERMEDIATE software level:
- Maintain standard technical explanations
- Add helpful tips for common pitfalls
- Reference connections to familiar programming concepts

For ADVANCED software level:
- Include performance considerations and best practices
- Add references to underlying implementation details
- Mention advanced alternatives or optimizations

For BEGINNER hardware level:
- Explain hardware terms when introduced (sensors, actuators, etc.)
- Add context about why hardware concepts matter
- Use simple analogies for physical concepts

For INTERMEDIATE hardware level:
- Reference common hardware knowledge
- Include practical setup tips
- Connect to real-world applications

For ADVANCED hardware level:
- Include technical specifications where relevant
- Discuss hardware limitations and workarounds
- Reference advanced integration patterns

CRITICAL RULES:
1. PRESERVE all code blocks exactly as they are - do not modify code
2. PRESERVE all markdown formatting (headers, lists, links, images)
3. PRESERVE all technical accuracy - do not simplify to the point of being incorrect
4. PRESERVE the original structure and flow of the content
5. ADD contextual explanations, not remove content
6. Keep the content length similar (±20%) to the original

ORIGINAL CONTENT:
{content}

PERSONALIZED CONTENT:"""

    def _build_translation_prompt(
        self,
        content: str,
        target_language: str = "ur"
    ) -> str:
        """
        Build the prompt for translating content to Urdu.

        Preserves code blocks, technical terms, and formatting.
        """
        language_name = "Urdu" if target_language == "ur" else target_language

        return f"""You are an expert translator specializing in technical and educational content. Translate the following content to {language_name}.

TRANSLATION GUIDELINES:

1. PRESERVE all code blocks exactly as they are - do not translate code
2. PRESERVE all markdown formatting (headers, lists, links, images)
3. PRESERVE technical terms that are commonly used in English (ROS, Python, Linux, etc.)
4. PRESERVE all URLs, file paths, and command-line instructions
5. Translate explanatory text naturally while maintaining technical accuracy
6. Use formal {language_name} appropriate for educational content
7. Keep the same document structure

For Urdu specifically:
- Use technical terms in their original English form when commonly understood
- Add transliteration in parentheses for complex terms if helpful
- Maintain right-to-left text flow for the translated portions

ORIGINAL CONTENT:
{content}

TRANSLATED CONTENT IN {language_name.upper()}:"""

    def personalize_content(
        self,
        content: str,
        user_profile: UserProfile,
    ) -> Dict[str, Any]:
        """
        Personalize chapter content based on user's background levels.

        Args:
            content: Original markdown content
            user_profile: User's software and hardware background levels

        Returns:
            Dict with personalized content and metadata
        """
        prompt = self._build_personalization_prompt(content, user_profile)

        try:
            response = self.cohere_client.chat(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
            )

            personalized = response.message.content[0].text.strip()
            now = datetime.utcnow()

            return {
                "content": personalized,
                "content_type": ContentType.PERSONALIZED,
                "original_hash": self._compute_content_hash(content),
                "generated_at": now,
                "expires_at": now + timedelta(days=self.cache_expiry_days),
                "language": "en"
            }

        except Exception as e:
            raise Exception(f"Cohere API error during personalization: {str(e)}")

    def translate_content(
        self,
        content: str,
        target_language: str = "ur"
    ) -> Dict[str, Any]:
        """
        Translate chapter content to the target language (default: Urdu).

        Args:
            content: Content to translate (original or personalized)
            target_language: ISO 639-1 language code (default: "ur" for Urdu)

        Returns:
            Dict with translated content and metadata
        """
        prompt = self._build_translation_prompt(content, target_language)

        try:
            response = self.cohere_client.chat(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.2,  # Lower temperature for translation
            )

            translated = response.message.content[0].text.strip()
            now = datetime.utcnow()

            return {
                "content": translated,
                "content_type": ContentType.TRANSLATED,
                "original_hash": self._compute_content_hash(content),
                "generated_at": now,
                "expires_at": now + timedelta(days=self.cache_expiry_days),
                "language": target_language
            }

        except Exception as e:
            raise Exception(f"Cohere API error during translation: {str(e)}")

    def personalize_and_translate(
        self,
        content: str,
        user_profile: UserProfile,
        target_language: str = "ur"
    ) -> Dict[str, Any]:
        """
        First personalize content, then translate to target language.

        Args:
            content: Original markdown content
            user_profile: User's background levels
            target_language: Target language for translation

        Returns:
            Dict with personalized and translated content
        """
        # Step 1: Personalize
        personalized_result = self.personalize_content(content, user_profile)

        # Step 2: Translate the personalized content
        translated_result = self.translate_content(
            personalized_result["content"],
            target_language
        )

        now = datetime.utcnow()

        return {
            "content": translated_result["content"],
            "content_type": ContentType.BOTH,
            "original_hash": self._compute_content_hash(content),
            "generated_at": now,
            "expires_at": now + timedelta(days=self.cache_expiry_days),
            "language": target_language
        }


# Singleton instance
personalization_service = PersonalizationService()
