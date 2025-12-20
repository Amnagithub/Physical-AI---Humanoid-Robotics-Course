# File: docs/modules/module-4/examples/llm_reasoning.py

import openai
from utils import load_api_key
import json
import datetime
from typing import Dict, Any, List

class LLMReasoningSystem:
    def __init__(self):
        api_key = load_api_key()
        if api_key:
            openai.api_key = api_key
        else:
            raise ValueError("OpenAI API key not found")

    def reason_about_goal(self, goal: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Perform reasoning about a goal using LLM
        """
        prompt = self._construct_reasoning_prompt(goal, context)

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are an intelligent assistant that performs cognitive reasoning for robotic tasks. Analyze goals and contexts, then provide reasoning results as JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.5,
                max_tokens=800
            )

            reasoning_result = self._parse_reasoning_response(response.choices[0].message.content)
            return reasoning_result

        except Exception as e:
            print(f"Error in LLM reasoning: {e}")
            return self._create_default_reasoning_result()

    def _construct_reasoning_prompt(self, goal: str, context: Dict[str, Any]) -> str:
        """
        Construct a prompt for cognitive reasoning
        """
        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        prompt = f"""
Perform cognitive reasoning about the following goal:

Goal: "{goal}"

Current Time: {current_time}

Context: {json.dumps(context, indent=2)}

Analyze the goal and context to provide reasoning in the following areas:
1. Goal Understanding: What is the real intent behind this request?
2. Precondition Analysis: What conditions must be true to start?
3. Resource Requirements: What resources are needed?
4. Risk Assessment: What risks are involved?
5. Alternative Strategies: Are there other ways to achieve this?
6. Success Criteria: How will we know the goal is achieved?

Provide your analysis as a JSON object with these fields:
- goal_intent: The understood intent
- preconditions: List of required preconditions
- resources_needed: List of required resources
- risks: List of potential risks
- alternative_strategies: List of alternative approaches
- success_criteria: List of success indicators
- confidence: Confidence level (0-1)
- reasoning_trace: Step-by-step reasoning process

Be thorough but practical in your analysis.
"""
        return prompt

    def _parse_reasoning_response(self, response_text: str) -> Dict[str, Any]:
        """
        Parse the LLM reasoning response
        """
        try:
            # Extract JSON from response
            if "```json" in response_text:
                start = response_text.find("```json") + 7
                end = response_text.find("```", start)
                json_str = response_text[start:end].strip()
            elif "```" in response_text:
                start = response_text.find("```") + 3
                end = response_text.find("```", start)
                json_str = response_text[start:end].strip()
            else:
                json_str = response_text.strip()

            result = json.loads(json_str)

            # Validate required fields
            required_fields = [
                "goal_intent", "preconditions", "resources_needed",
                "risks", "alternative_strategies", "success_criteria",
                "confidence", "reasoning_trace"
            ]

            for field in required_fields:
                if field not in result:
                    result[field] = [] if field in [
                        "preconditions", "resources_needed", "risks",
                        "alternative_strategies", "success_criteria", "reasoning_trace"
                    ] else ("unknown" if field == "goal_intent" else 0.5)

            return result

        except json.JSONDecodeError:
            print(f"Could not parse reasoning response as JSON: {response_text}")
            return self._create_default_reasoning_result()
        except Exception as e:
            print(f"Error parsing reasoning response: {e}")
            return self._create_default_reasoning_result()

    def _create_default_reasoning_result(self) -> Dict[str, Any]:
        """
        Create a default reasoning result when LLM fails
        """
        return {
            "goal_intent": "unknown",
            "preconditions": [],
            "resources_needed": [],
            "risks": [],
            "alternative_strategies": [],
            "success_criteria": [],
            "confidence": 0.5,
            "reasoning_trace": ["LLM reasoning failed, using default response"]
        }

    def apply_reasoning_to_plan(self, plan: List[Dict[str, Any]], reasoning: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Apply reasoning results to improve a plan
        """
        improved_plan = plan.copy()

        # Add preconditions as initial checks
        for precondition in reasoning.get("preconditions", []):
            improved_plan.insert(0, {
                "action": "check_condition",
                "parameters": {"condition": precondition},
                "description": f"Verify: {precondition}",
                "required": True
            })

        # Add risk mitigations
        for risk in reasoning.get("risks", []):
            # In a real system, this would add specific risk mitigation actions
            print(f"Consider adding mitigation for risk: {risk}")

        # Add success verification
        for criterion in reasoning.get("success_criteria", []):
            improved_plan.append({
                "action": "verify_success",
                "parameters": {"criterion": criterion},
                "description": f"Verify success: {criterion}",
                "required": True
            })

        return improved_plan

    def demo_reasoning(self):
        """
        Demonstrate the LLM reasoning system
        """
        print("LLM Reasoning System Demo")
        print("=" * 40)

        # Example goals
        test_goals = [
            "Please bring me a glass of water from the kitchen",
            "Clean the dining table before dinner"
        ]

        for i, goal in enumerate(test_goals, 1):
            print(f"\nGoal {i}: {goal}")

            context = {
                "robot_capabilities": ["navigation", "manipulation", "perception"],
                "environment": "home with kitchen, dining room, living room",
                "time_of_day": "afternoon",
                "user_preferences": {"preferred_speed": "careful", "safety_priority": "high"}
            }

            reasoning = self.reason_about_goal(goal, context)

            print(f"  Goal Intent: {reasoning['goal_intent']}")
            print(f"  Confidence: {reasoning['confidence']:.2f}")
            print(f"  Precondition Count: {len(reasoning['preconditions'])}")
            print(f"  Risk Count: {len(reasoning['risks'])}")

            # Show first few preconditions and risks
            if reasoning['preconditions']:
                print(f"  Precondition Examples: {reasoning['preconditions'][:2]}")
            if reasoning['risks']:
                print(f"  Risk Examples: {reasoning['risks'][:2]}")

if __name__ == "__main__":
    reasoning_system = LLMReasoningSystem()
    reasoning_system.demo_reasoning()