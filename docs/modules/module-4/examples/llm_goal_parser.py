# File: docs/modules/module-4/examples/llm_goal_parser.py

import openai
from utils import load_api_key
import json
import datetime
from typing import Dict, Any

class LLMGoalParser:
    def __init__(self):
        api_key = load_api_key()
        if api_key:
            openai.api_key = api_key
        else:
            raise ValueError("OpenAI API key not found")

    def parse_goal(self, goal_text: str, context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Parse a natural language goal using an LLM
        """
        if context is None:
            context = {
                "robot_capabilities": ["navigation", "manipulation", "perception"],
                "environment": "home environment",
                "time_of_day": datetime.datetime.now().strftime("%H:%M")
            }

        prompt = f"""
Parse the following natural language goal into structured components:

Goal: "{goal_text}"

Context:
- Robot capabilities: {context['robot_capabilities']}
- Environment: {context['environment']}
- Current time: {context['time_of_day']}

Provide the parsed goal as a JSON object with these fields:
- intent: The main purpose of the goal
- objects: List of relevant objects
- locations: List of relevant locations
- constraints: Any constraints or preferences
- required_capabilities: List of capabilities needed

Example response format:
{{
  "intent": "fetch_item",
  "objects": ["water", "glass"],
  "locations": ["kitchen"],
  "constraints": ["carefully"],
  "required_capabilities": ["navigation", "manipulation"]
}}
"""

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that parses natural language goals for robots. Respond with structured JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.2,
                max_tokens=500
            )

            # Parse the response
            content = response.choices[0].message.content

            # Extract JSON if wrapped in code blocks
            if "```json" in content:
                start = content.find("```json") + 7
                end = content.find("```", start)
                json_str = content[start:end].strip()
            elif "```" in content:
                start = content.find("```") + 3
                end = content.find("```", start)
                json_str = content[start:end].strip()
            else:
                json_str = content.strip()

            parsed_goal = json.loads(json_str)
            return parsed_goal

        except Exception as e:
            print(f"Error parsing goal with LLM: {e}")
            # Return a basic fallback parsing
            return self._fallback_parse(goal_text)

    def _fallback_parse(self, goal_text: str) -> Dict[str, Any]:
        """
        Fallback parsing if LLM fails
        """
        text_lower = goal_text.lower()

        # Simple keyword-based parsing
        intents = {
            "fetch": ["bring", "get", "fetch", "pick up", "take"],
            "navigate": ["go to", "move to", "navigate to", "walk to"],
            "clean": ["clean", "tidy", "organize", "clear"],
            "monitor": ["check", "look at", "inspect", "examine"],
            "communicate": ["tell", "speak", "say", "announce"]
        }

        intent = "unknown"
        for intent_name, keywords in intents.items():
            if any(keyword in text_lower for keyword in keywords):
                intent = intent_name
                break

        return {
            "intent": intent,
            "objects": [],
            "locations": [],
            "constraints": [],
            "required_capabilities": ["navigation"] if intent != "unknown" else []
        }

    def demo_parsing(self):
        """
        Demonstrate the LLM goal parsing
        """
        print("LLM Goal Parser Demo")
        print("=" * 30)

        test_goals = [
            "Please bring me a glass of water from the kitchen",
            "Go to the living room and clean the table",
            "Check if the door is locked",
            "Tell me what time it is"
        ]

        for i, goal in enumerate(test_goals, 1):
            print(f"\nTest {i}: '{goal}'")

            parsed = self.parse_goal(goal)

            print(f"  Intent: {parsed['intent']}")
            print(f"  Objects: {parsed['objects']}")
            print(f"  Locations: {parsed['locations']}")
            print(f"  Constraints: {parsed['constraints']}")
            print(f"  Required Capabilities: {parsed['required_capabilities']}")

if __name__ == "__main__":
    parser = LLMGoalParser()
    parser.demo_parsing()