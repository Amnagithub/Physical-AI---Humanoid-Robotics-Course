---
title: "Chapter 2: Cognitive Planning with Large Language Models"
sidebar_label: "Chapter 2: LLM Planning"
---

# Chapter 2: Cognitive Planning with Large Language Models

## Overview

This chapter explores cognitive planning with Large Language Models (LLMs) for humanoid robots. You'll learn how to translate natural language goals into structured ROS 2 action sequences using LLMs for reasoning and planning.

## Learning Objectives

After completing this chapter, you will be able to:
- Parse natural language goals into structured task sequences
- Design LLM-based task decomposition systems
- Generate ROS 2 action sequences from LLM outputs
- Implement cognitive reasoning with LLMs
- Apply safety and validation to LLM-generated plans

## 1. Natural Language Goal Parsing

### 1.1 Understanding Natural Language Goals

Natural language goals are high-level instructions provided in human-readable form, such as "Please bring me a glass of water from the kitchen" or "Clean the living room table." These goals need to be parsed into structured, executable robot behaviors.

### 1.2 Challenges in Natural Language Understanding

Robotic applications of natural language processing face several challenges:

- **Ambiguity**: Natural language is often ambiguous ("the ball" - which ball?)
- **Context Dependency**: Meaning depends on environmental context
- **Implicit Information**: Humans often leave out explicit details
- **Robustness**: Must handle variations in phrasing and grammar
- **Real-time Processing**: Need for timely response to user requests

### 1.3 Goal Representation

For effective processing, natural language goals should be represented in a structured format:

- **Intent**: The primary goal (e.g., "fetch", "navigate", "clean")
- **Objects**: Entities involved (e.g., "glass", "water", "kitchen")
- **Constraints**: Limitations or preferences (e.g., "carefully", "quickly")
- **Context**: Environmental or situational information

### 1.4 Implementation Example

```python
from dataclasses import dataclass
from typing import Dict, List, Optional
import re

@dataclass
class ParsedGoal:
    """
    Data class representing a parsed natural language goal
    """
    intent: str
    objects: List[str]
    locations: List[str]
    constraints: List[str]
    original_text: str
    confidence: float

class NaturalLanguageGoalParser:
    def __init__(self):
        # Define common intents and their patterns
        self.intent_patterns = {
            "fetch": [r"bring.*", r"get.*", r"fetch.*", r"pick.*up", r"take.*"],
            "navigate": [r"go.*to", r"move.*to", r"navigate.*to", r"walk.*to"],
            "clean": [r"clean.*", r"tidy.*", r"organize.*", r"clear.*"],
            "monitor": [r"check.*", r"look.*at", r"inspect.*", r"examine.*"],
            "communicate": [r"tell.*", r"speak.*", r"say.*", r"announce.*"]
        }

        # Define common objects and locations
        self.objects = [
            "glass", "water", "cup", "book", "phone", "bottle",
            "ball", "table", "chair", "box", "plate", "food"
        ]

        self.locations = [
            "kitchen", "living room", "bedroom", "office",
            "bathroom", "hallway", "dining room", "garage"
        ]

    def parse_goal(self, text: str) -> Optional[ParsedGoal]:
        """
        Parse a natural language goal into structured format
        """
        text_lower = text.lower().strip()

        # Extract intent
        intent = self._extract_intent(text_lower)

        # Extract objects
        objects = self._extract_entities(text_lower, self.objects)

        # Extract locations
        locations = self._extract_entities(text_lower, self.locations)

        # Extract constraints (adverbs, adjectives)
        constraints = self._extract_constraints(text_lower)

        # Calculate confidence based on how much of the goal was understood
        confidence = self._calculate_confidence(intent, objects, locations)

        return ParsedGoal(
            intent=intent,
            objects=objects,
            locations=locations,
            constraints=constraints,
            original_text=text,
            confidence=confidence
        )

    def _extract_intent(self, text: str) -> str:
        """
        Extract the primary intent from the text
        """
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text):
                    return intent
        return "unknown"

    def _extract_entities(self, text: str, entity_list: List[str]) -> List[str]:
        """
        Extract specific entities from the text
        """
        found_entities = []
        for entity in entity_list:
            if entity in text:
                found_entities.append(entity)
        return found_entities

    def _extract_constraints(self, text: str) -> List[str]:
        """
        Extract constraints from the text (adverbs, adjectives)
        """
        constraint_keywords = [
            "carefully", "quickly", "slowly", "gently", "carefully",
            "immediately", "urgently", "quietly", "loudly", "safely"
        ]

        constraints = []
        for keyword in constraint_keywords:
            if keyword in text:
                constraints.append(keyword)

        return constraints

    def _calculate_confidence(self, intent: str, objects: List[str], locations: List[str]) -> float:
        """
        Calculate confidence based on how much of the goal was understood
        """
        score = 0

        # Intent contributes to confidence
        if intent != "unknown":
            score += 0.4

        # Objects contribute to confidence
        score += min(0.3, len(objects) * 0.1)

        # Locations contribute to confidence
        score += min(0.3, len(locations) * 0.1)

        return min(1.0, score)

# Example usage
parser = NaturalLanguageGoalParser()
goal = parser.parse_goal("Please carefully bring me a glass of water from the kitchen")
print(f"Intent: {goal.intent}")
print(f"Objects: {goal.objects}")
print(f"Locations: {goal.locations}")
print(f"Constraints: {goal.constraints}")
print(f"Confidence: {goal.confidence}")
```

## 2. LLM-Based Task Decomposition

### 2.1 Introduction to Task Decomposition

Task decomposition is the process of breaking down complex goals into simpler, executable subtasks. LLMs excel at this by understanding the relationships between different actions and the logical sequence required to achieve a goal.

### 2.2 Decomposition Strategies

Effective task decomposition strategies include:

- **Hierarchical Decomposition**: Breaking tasks into levels of abstraction
- **Functional Decomposition**: Grouping tasks by function or capability
- **Temporal Decomposition**: Sequencing tasks based on time dependencies
- **Spatial Decomposition**: Organizing tasks based on location

### 2.3 LLM Prompt Engineering for Task Decomposition

The quality of task decomposition depends heavily on how the LLM is prompted:

- **Clear Context**: Provide information about robot capabilities and environment
- **Structured Output**: Request output in a specific format
- **Examples**: Include examples of good decompositions
- **Constraints**: Specify limitations or requirements

### 2.4 Implementation Example

```python
import openai
from typing import List, Dict, Any
from utils import LLMPlan, load_api_key
import json

class LLMTaskDecomposer:
    def __init__(self):
        api_key = load_api_key()
        if api_key:
            openai.api_key = api_key
        else:
            raise ValueError("OpenAI API key not found")

    def decompose_task(self, goal: str, context: Dict[str, Any]) -> LLMPlan:
        """
        Use LLM to decompose a high-level goal into a sequence of subtasks
        """
        # Construct the prompt for the LLM
        prompt = self._construct_decomposition_prompt(goal, context)

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",  # or gpt-4 for better results
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that decomposes complex tasks into sequences of robot actions. Respond with a JSON array of action objects."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=1000
            )

            # Parse the LLM response
            plan_data = self._parse_llm_response(response.choices[0].message.content)

            # Create LLMPlan object
            plan = LLMPlan(
                id=f"plan_{int(datetime.now().timestamp())}",
                goal=goal,
                plan=plan_data,
                confidence=0.85,  # Default confidence, could be calculated from response
                generated_at=datetime.now()
            )

            return plan

        except Exception as e:
            print(f"Error in LLM task decomposition: {e}")
            # Return a fallback plan
            return self._create_fallback_plan(goal)

    def _construct_decomposition_prompt(self, goal: str, context: Dict[str, Any]) -> str:
        """
        Construct a prompt for the LLM to decompose tasks
        """
        robot_capabilities = context.get("robot_capabilities", ["navigation", "manipulation", "perception"])
        environment = context.get("environment", "unknown")

        prompt = f"""
Decompose the following goal into a sequence of executable robot actions:

Goal: "{goal}"

Robot Capabilities: {', '.join(robot_capabilities)}
Environment: {environment}

Provide the decomposition as a JSON array where each element has:
- action: The action type (e.g., "navigate", "grasp", "perceive", "transport")
- parameters: Relevant parameters for the action
- description: Brief description of what the action does

Example format:
[
  {{
    "action": "navigate",
    "parameters": {{"target_location": "kitchen"}},
    "description": "Move to the kitchen area"
  }},
  {{
    "action": "perceive",
    "parameters": {{"target_object": "glass"}},
    "description": "Look for a glass in the kitchen"
  }}
]

Ensure the sequence is logical and all required preconditions are met before each action.
"""
        return prompt

    def _parse_llm_response(self, response_text: str) -> List[Dict[str, Any]]:
        """
        Parse the LLM response into a structured plan
        """
        try:
            # Try to extract JSON from the response
            # Sometimes LLMs wrap JSON in markdown code blocks
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

            plan = json.loads(json_str)

            # Validate the plan structure
            if not isinstance(plan, list):
                raise ValueError("LLM response is not a list")

            for item in plan:
                if not isinstance(item, dict):
                    raise ValueError("Plan items must be dictionaries")
                if "action" not in item:
                    raise ValueError("Plan items must have 'action' field")

            return plan

        except json.JSONDecodeError:
            print(f"Could not parse LLM response as JSON: {response_text}")
            return []
        except ValueError as e:
            print(f"Invalid plan structure: {e}")
            return []

    def _create_fallback_plan(self, goal: str) -> LLMPlan:
        """
        Create a fallback plan when LLM decomposition fails
        """
        # Simple fallback plan
        fallback_plan = [
            {
                "action": "communication",
                "parameters": {"message": f"Unable to process goal: {goal}"},
                "description": "Communicate failure to user"
            }
        ]

        return LLMPlan(
            id=f"fallback_{int(datetime.now().timestamp())}",
            goal=goal,
            plan=fallback_plan,
            confidence=0.1,
            generated_at=datetime.now()
        )

# Example usage
decomposer = LLMTaskDecomposer()
context = {
    "robot_capabilities": ["navigation", "manipulation", "perception"],
    "environment": "home environment with kitchen, living room, and bedroom"
}
plan = decomposer.decompose_task("Please bring me a glass of water from the kitchen", context)
print(f"Generated plan: {json.dumps(plan.plan, indent=2)}")
```

## 3. ROS 2 Action Sequence Generation

### 3.1 Converting LLM Plans to ROS 2 Actions

Once an LLM has decomposed a goal into a sequence of subtasks, these need to be converted into actual ROS 2 actions that the robot can execute. This involves:

- Mapping high-level actions to specific ROS 2 action types
- Converting parameters to the appropriate ROS 2 message formats
- Ensuring action sequences respect dependencies and constraints
- Adding safety checks and validation

### 3.2 Action Mapping Strategies

Different strategies can be used for mapping:

- **Direct Mapping**: One-to-one mapping between LLM actions and ROS 2 actions
- **Pattern Matching**: Using templates to map common action patterns
- **Capability-Based**: Mapping based on robot capabilities
- **Context-Aware**: Mapping that considers environmental context

### 3.3 Implementation Example

```python
from utils import Action, ActionSequence
from typing import Dict, Any, List
import re

class LLMPlanToROS2Converter:
    def __init__(self):
        # Define mapping between LLM actions and ROS 2 action types
        self.action_mappings = {
            "navigate": self._convert_navigate,
            "grasp": self._convert_grasp,
            "perceive": self._convert_perceive,
            "transport": self._convert_transport,
            "communicate": self._convert_communicate,
            "inspect": self._convert_inspect,
            "clean": self._convert_clean
        }

    def convert_plan_to_ros2_actions(self, llm_plan: LLMPlan) -> ActionSequence:
        """
        Convert an LLM-generated plan to ROS 2 action sequence
        """
        ros2_actions = []

        for i, llm_action in enumerate(llm_plan.plan):
            action_type = llm_action.get("action", "unknown")

            if action_type in self.action_mappings:
                ros2_action = self.action_mappings[action_type](llm_action, f"action_{i}")
                if ros2_action:
                    ros2_actions.append(ros2_action)
            else:
                print(f"Unknown action type: {action_type}, skipping...")

        return ActionSequence(
            id=f"ros2_{llm_plan.id}",
            steps=ros2_actions,
            status="pending",
            created_at=llm_plan.generated_at,
            robot_id="robot_001"
        )

    def _convert_navigate(self, llm_action: Dict[str, Any], action_id: str) -> Action:
        """
        Convert LLM navigate action to ROS 2 navigation action
        """
        params = llm_action.get("parameters", {})
        target_location = params.get("target_location", "unknown")

        # Convert location to coordinates (this would use a map in a real system)
        location_coords = self._get_coordinates_for_location(target_location)

        return Action(
            id=f"nav_{action_id}",
            type="navigation",
            parameters={
                "target_pose": location_coords,
                "location_name": target_location
            },
            priority=1,
            timeout=120
        )

    def _convert_grasp(self, llm_action: Dict[str, Any], action_id: str) -> Action:
        """
        Convert LLM grasp action to ROS 2 manipulation action
        """
        params = llm_action.get("parameters", {})
        target_object = params.get("target_object", "unknown_object")

        return Action(
            id=f"grasp_{action_id}",
            type="manipulation",
            parameters={
                "object_name": target_object,
                "action_type": "grasp",
                "grasp_type": "pinch"  # Default grasp type
            },
            priority=2,
            timeout=60
        )

    def _convert_perceive(self, llm_action: Dict[str, Any], action_id: str) -> Action:
        """
        Convert LLM perceive action to ROS 2 perception action
        """
        params = llm_action.get("parameters", {})
        target_object = params.get("target_object", "unknown_object")

        return Action(
            id=f"percept_{action_id}",
            type="perception",
            parameters={
                "target_object": target_object,
                "action_type": "detection"
            },
            priority=1,
            timeout=30
        )

    def _convert_transport(self, llm_action: Dict[str, Any], action_id: str) -> Action:
        """
        Convert LLM transport action to sequence of navigation and manipulation
        """
        params = llm_action.get("parameters", {})
        target_location = params.get("target_location", "unknown")
        object_to_transport = params.get("object", "unknown_object")

        # For transport, we need to navigate, grasp, then navigate again
        # This would typically be a compound action
        return Action(
            id=f"transport_{action_id}",
            type="manipulation",
            parameters={
                "action_type": "transport",
                "object_name": object_to_transport,
                "destination": target_location
            },
            priority=3,
            timeout=180
        )

    def _convert_communicate(self, llm_action: Dict[str, Any], action_id: str) -> Action:
        """
        Convert LLM communicate action to ROS 2 communication action
        """
        params = llm_action.get("parameters", {})
        message = params.get("message", "Hello")

        return Action(
            id=f"comm_{action_id}",
            type="communication",
            parameters={
                "message": message,
                "action_type": "speak"
            },
            priority=0,
            timeout=10
        )

    def _convert_inspect(self, llm_action: Dict[str, Any], action_id: str) -> Action:
        """
        Convert LLM inspect action to ROS 2 perception action
        """
        params = llm_action.get("parameters", {})
        target_object = params.get("target_object", "unknown_object")

        return Action(
            id=f"inspect_{action_id}",
            type="perception",
            parameters={
                "target_object": target_object,
                "action_type": "inspection",
                "inspection_type": "visual"
            },
            priority=1,
            timeout=45
        )

    def _convert_clean(self, llm_action: Dict[str, Any], action_id: str) -> Action:
        """
        Convert LLM clean action to ROS 2 manipulation action
        """
        params = llm_action.get("parameters", {})
        target_area = params.get("target_area", "unknown_area")

        return Action(
            id=f"clean_{action_id}",
            type="manipulation",
            parameters={
                "action_type": "cleaning",
                "target_area": target_area,
                "cleaning_tool": "generic_cleaner"
            },
            priority=2,
            timeout=150
        )

    def _get_coordinates_for_location(self, location_name: str) -> Dict[str, float]:
        """
        Get coordinates for a named location (simplified mapping)
        """
        location_map = {
            "kitchen": {"x": 1.0, "y": 2.0, "z": 0.0},
            "living room": {"x": 3.0, "y": 4.0, "z": 0.0},
            "bedroom": {"x": 5.0, "y": 6.0, "z": 0.0},
            "office": {"x": 7.0, "y": 8.0, "z": 0.0},
            "dining room": {"x": 2.0, "y": 5.0, "z": 0.0}
        }

        return location_map.get(location_name.lower(), {"x": 0.0, "y": 0.0, "z": 0.0})

# Example usage
converter = LLMPlanToROS2Converter()

# Example LLM plan
example_plan = LLMPlan(
    id="example_plan_1",
    goal="Bring a glass of water from kitchen",
    plan=[
        {"action": "navigate", "parameters": {"target_location": "kitchen"}, "description": "Go to kitchen"},
        {"action": "perceive", "parameters": {"target_object": "glass"}, "description": "Find glass"},
        {"action": "grasp", "parameters": {"target_object": "glass"}, "description": "Pick up glass"},
        {"action": "navigate", "parameters": {"target_location": "living room"}, "description": "Return to living room"}
    ],
    confidence=0.9,
    generated_at=datetime.now()
)

ros2_sequence = converter.convert_plan_to_ros2_actions(example_plan)
print(f"Converted {len(ros2_sequence.steps)} actions to ROS 2 format")
for i, action in enumerate(ros2_sequence.steps):
    print(f"  {i+1}. {action.type}: {action.parameters}")
```

## 4. Cognitive Reasoning with LLMs

### 4.1 Introduction to Cognitive Reasoning

Cognitive reasoning in robotics involves higher-level thinking processes that go beyond simple command execution. LLMs can provide:

- **Common-sense reasoning**: Understanding implicit relationships
- **Planning under uncertainty**: Adapting plans when conditions change
- **Learning from experience**: Improving with repeated interactions
- **Context awareness**: Understanding situational context

### 4.2 Reasoning Capabilities

LLMs can enhance robotic reasoning in several ways:

- **Spatial reasoning**: Understanding locations and relationships
- **Temporal reasoning**: Understanding sequences and timing
- **Causal reasoning**: Understanding cause-and-effect relationships
- **Social reasoning**: Understanding human intentions and preferences

### 4.3 Implementation Example

```python
import openai
from typing import Dict, Any, List
from utils import load_api_key
import datetime

class LLMCognitiveReasoner:
    def __init__(self):
        api_key = load_api_key()
        if api_key:
            openai.api_key = api_key
        else:
            raise ValueError("OpenAI API key not found")

    def reason_about_plan(self, plan: List[Dict[str, Any]], context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Use LLM to reason about a plan and suggest improvements or alternatives
        """
        prompt = self._construct_reasoning_prompt(plan, context)

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that performs cognitive reasoning for robot planning. Analyze the plan and context, then provide reasoning results as JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.4,
                max_tokens=800
            )

            reasoning_result = self._parse_reasoning_response(response.choices[0].message.content)
            return reasoning_result

        except Exception as e:
            print(f"Error in LLM reasoning: {e}")
            return self._create_default_reasoning_result()

    def _construct_reasoning_prompt(self, plan: List[Dict[str, Any]], context: Dict[str, Any]) -> str:
        """
        Construct a prompt for cognitive reasoning
        """
        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        prompt = f"""
Perform cognitive reasoning about the following robot plan:

Current Time: {current_time}
Context: {json.dumps(context, indent=2)}

Plan: {json.dumps(plan, indent=2)}

Consider:
1. Are there any potential issues with this plan?
2. Are there better alternatives?
3. What environmental factors should be considered?
4. What safety concerns exist?
5. Are there dependencies between actions that need attention?

Provide your analysis as a JSON object with these fields:
- issues: List of potential issues
- suggestions: List of improvement suggestions
- risks: List of safety risks
- dependencies: List of action dependencies
- confidence: Overall confidence in the plan (0-1)

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
            required_fields = ["issues", "suggestions", "risks", "dependencies", "confidence"]
            for field in required_fields:
                if field not in result:
                    result[field] = [] if field in ["issues", "suggestions", "risks", "dependencies"] else 0.5

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
            "issues": [],
            "suggestions": [],
            "risks": [],
            "dependencies": [],
            "confidence": 0.5
        }

    def adapt_plan_based_on_reasoning(self, original_plan: List[Dict[str, Any]], reasoning: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Adapt the original plan based on the reasoning results
        """
        adapted_plan = original_plan.copy()

        # Apply suggestions to improve the plan
        for suggestion in reasoning.get("suggestions", []):
            # This would implement specific adaptation logic based on the suggestion
            print(f"Applying suggestion: {suggestion}")
            # In a real implementation, this would modify the plan based on the suggestion

        # Add safety checks based on identified risks
        for risk in reasoning.get("risks", []):
            print(f"Adding safety check for risk: {risk}")
            # In a real implementation, this would add safety actions to mitigate risks

        return adapted_plan

# Example usage
reasoner = LLMCognitiveReasoner()

# Example plan and context
example_plan = [
    {"action": "navigate", "parameters": {"target_location": "kitchen"}, "description": "Go to kitchen"},
    {"action": "grasp", "parameters": {"target_object": "glass"}, "description": "Pick up glass"}
]

example_context = {
    "robot_position": {"x": 0, "y": 0},
    "environment": "home with obstacles",
    "time_of_day": "evening",
    "user_preferences": {"preferred_speed": "careful"}
}

reasoning = reasoner.reason_about_plan(example_plan, example_context)
print(f"Reasoning result: {json.dumps(reasoning, indent=2)}")

adapted_plan = reasoner.adapt_plan_based_on_reasoning(example_plan, reasoning)
print(f"Adapted plan: {json.dumps(adapted_plan, indent=2)}")
```

## 5. Safety and Validation for LLM Plans

### 5.1 Importance of Safety Validation

LLM-generated plans must be validated for safety before execution. This includes:

- **Physical Safety**: Ensuring actions don't cause harm to people or property
- **Logical Safety**: Ensuring plans are coherent and executable
- **Contextual Safety**: Ensuring actions are appropriate for the situation
- **Capability Safety**: Ensuring the robot can actually perform the actions

### 5.2 Validation Layers

A comprehensive safety validation system includes:

- **Input Validation**: Validating the natural language goal
- **Plan Validation**: Checking the LLM-generated plan for safety
- **Action Validation**: Verifying each action is safe to execute
- **Runtime Validation**: Monitoring execution for safety issues

### 5.3 Implementation Example

```python
from utils import ActionSequence, validate_action_sequence
from typing import List, Dict, Any

class SafetyValidator:
    def __init__(self):
        # Define safety rules
        self.safety_rules = [
            self._check_navigation_safety,
            self._check_manipulation_safety,
            self._check_environmental_safety,
            self._check_robot_capability_safety
        ]

        # Define dangerous actions or patterns
        self.dangerous_patterns = [
            "self-harm",
            "harm to humans",
            "damage to property",
            "forbidden areas"
        ]

    def validate_plan(self, plan: ActionSequence, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate an LLM-generated plan for safety
        """
        validation_result = {
            "is_safe": True,
            "issues": [],
            "warnings": [],
            "safe_actions": [],
            "blocked_actions": []
        }

        # Check each action in the sequence
        for i, action in enumerate(plan.steps):
            action_result = self._validate_single_action(action, context)

            if action_result["is_safe"]:
                validation_result["safe_actions"].append(i)
            else:
                validation_result["is_safe"] = False
                validation_result["blocked_actions"].append(i)
                validation_result["issues"].extend(action_result["issues"])

        # Additional plan-level validation
        plan_level_issues = self._validate_plan_level(plan, context)
        validation_result["issues"].extend(plan_level_issues)

        return validation_result

    def _validate_single_action(self, action: Action, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate a single action for safety
        """
        result = {
            "is_safe": True,
            "issues": []
        }

        # Apply all safety rules
        for rule in self.safety_rules:
            rule_result = rule(action, context)
            if not rule_result["is_safe"]:
                result["is_safe"] = False
                result["issues"].extend(rule_result["issues"])

        return result

    def _check_navigation_safety(self, action: Action, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Check if navigation action is safe
        """
        result = {"is_safe": True, "issues": []}

        if action.type == "navigation":
            target_pose = action.parameters.get("target_pose", {})

            # Check if target is in forbidden area
            forbidden_areas = context.get("forbidden_areas", [])
            target_x = target_pose.get("x", 0)
            target_y = target_pose.get("y", 0)

            for area in forbidden_areas:
                if self._is_in_area((target_x, target_y), area):
                    result["is_safe"] = False
                    result["issues"].append(f"Navigation target {target_pose} is in forbidden area: {area}")

        return result

    def _check_manipulation_safety(self, action: Action, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Check if manipulation action is safe
        """
        result = {"is_safe": True, "issues": []}

        if action.type == "manipulation":
            object_name = action.parameters.get("object_name", "").lower()

            # Check if object is dangerous
            dangerous_objects = ["knife", "blade", "sharp object", "fire", "hot object"]
            if object_name in dangerous_objects:
                result["is_safe"] = False
                result["issues"].append(f"Manipulation of dangerous object: {object_name}")

        return result

    def _check_environmental_safety(self, action: Action, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Check environmental safety factors
        """
        result = {"is_safe": True, "issues": []}

        # Check if it's safe to operate in current conditions
        current_time = context.get("time_of_day", "day")
        people_present = context.get("people_in_area", True)
        lighting = context.get("lighting", "good")

        if current_time == "night" and lighting == "poor":
            result["issues"].append("Poor lighting conditions, extra caution needed")

        if not people_present and action.type == "communication":
            # Warn about communication when no people are present
            result["issues"].append("Communication action when no people detected in area")

        return result

    def _check_robot_capability_safety(self, action: Action, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Check if robot is capable of performing the action safely
        """
        result = {"is_safe": True, "issues": []}

        # Check robot capabilities
        robot_capabilities = context.get("robot_capabilities", [])
        action_type = action.type

        if action_type not in robot_capabilities:
            result["is_safe"] = False
            result["issues"].append(f"Robot lacks capability for action type: {action_type}")

        # Check action parameters are within robot limits
        if action_type == "navigation":
            target_pose = action.parameters.get("target_pose", {})
            # Check if target is within robot's operational range
            max_range = context.get("robot_max_range", 10.0)  # meters

            target_x = abs(target_pose.get("x", 0))
            target_y = abs(target_pose.get("y", 0))
            distance = (target_x**2 + target_y**2)**0.5

            if distance > max_range:
                result["is_safe"] = False
                result["issues"].append(f"Navigation target too far: {distance:.2f}m (max: {max_range}m)")

        return result

    def _is_in_area(self, point: tuple, area: Dict[str, Any]) -> bool:
        """
        Check if a point is within a defined area
        """
        # Simple rectangular area check
        if "min_x" in area and "max_x" in area and "min_y" in area and "max_y" in area:
            x, y = point
            return (area["min_x"] <= x <= area["max_x"] and
                    area["min_y"] <= y <= area["max_y"])
        return False

    def _validate_plan_level(self, plan: ActionSequence, context: Dict[str, Any]) -> List[str]:
        """
        Perform plan-level safety validation
        """
        issues = []

        # Check for dangerous action sequences
        action_types = [action.type for action in plan.steps]

        # Check for potentially dangerous combinations
        if "navigation" in action_types and "manipulation" in action_types:
            # More complex validation would check the sequence and timing
            pass

        # Check plan duration against safety limits
        estimated_duration = self._estimate_plan_duration(plan)
        max_duration = context.get("max_plan_duration", 300)  # 5 minutes default

        if estimated_duration > max_duration:
            issues.append(f"Plan duration {estimated_duration}s exceeds safety limit {max_duration}s")

        return issues

    def _estimate_plan_duration(self, plan: ActionSequence) -> float:
        """
        Estimate the total duration of a plan
        """
        total_time = 0.0

        for action in plan.steps:
            # Use timeout as a rough estimate of action duration
            total_time += action.timeout

        return total_time

# Example usage
validator = SafetyValidator()

# Example action sequence
test_sequence = ActionSequence(
    id="test_seq_1",
    steps=[
        Action(
            id="nav_1",
            type="navigation",
            parameters={"target_pose": {"x": 1.0, "y": 2.0}},
            priority=1,
            timeout=60
        ),
        Action(
            id="grasp_1",
            type="manipulation",
            parameters={"object_name": "cup"},
            priority=2,
            timeout=30
        )
    ],
    status="pending",
    created_at=datetime.now(),
    robot_id="robot_001"
)

context = {
    "forbidden_areas": [{"min_x": 10, "max_x": 20, "min_y": 10, "max_y": 20}],
    "robot_capabilities": ["navigation", "manipulation"],
    "robot_max_range": 15.0,
    "max_plan_duration": 300
}

validation_result = validator.validate_plan(test_sequence, context)
print(f"Validation result: {json.dumps(validation_result, indent=2)}")
```

## 6. Practical Examples for LLM-Based Planning

### 6.1 LLM Goal Parser Example

```python
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
```

### 6.2 Task Sequencer Example

```python
# File: docs/modules/module-4/examples/task_sequencer.py

from utils import Action, ActionSequence
from typing import List, Dict, Any
import datetime

class TaskSequencer:
    def __init__(self):
        # Define common task patterns
        self.task_patterns = {
            "fetch_item": self._sequence_fetch_item,
            "navigate_and_report": self._sequence_navigate_and_report,
            "clean_area": self._sequence_clean_area,
            "inspect_object": self._sequence_inspect_object
        }

    def sequence_tasks(self, parsed_goal: Dict[str, Any], robot_context: Dict[str, Any]) -> ActionSequence:
        """
        Generate an action sequence based on parsed goal and robot context
        """
        intent = parsed_goal.get("intent", "unknown")

        if intent in self.task_patterns:
            actions = self.task_patterns[intent](parsed_goal, robot_context)
        else:
            actions = self._default_sequence(parsed_goal, robot_context)

        return ActionSequence(
            id=f"seq_{int(datetime.datetime.now().timestamp())}",
            steps=actions,
            status="pending",
            created_at=datetime.datetime.now(),
            robot_id=robot_context.get("robot_id", "robot_001")
        )

    def _sequence_fetch_item(self, parsed_goal: Dict[str, Any], robot_context: Dict[str, Any]) -> List[Action]:
        """
        Sequence for fetching an item
        """
        actions = []

        # Navigate to location
        locations = parsed_goal.get("locations", ["unknown"])
        if locations:
            actions.append(Action(
                id=f"nav_to_{locations[0]}_{int(datetime.datetime.now().timestamp())}",
                type="navigation",
                parameters={
                    "target_location": locations[0],
                    "location_name": locations[0]
                },
                priority=1,
                timeout=120
            ))

        # Find object
        objects = parsed_goal.get("objects", ["unknown"])
        if objects:
            actions.append(Action(
                id=f"find_{objects[0]}_{int(datetime.datetime.now().timestamp())}",
                type="perception",
                parameters={
                    "target_object": objects[0],
                    "action_type": "detection"
                },
                priority=2,
                timeout=45
            ))

        # Grasp object
        actions.append(Action(
            id=f"grasp_{objects[0] if objects else 'object'}_{int(datetime.datetime.now().timestamp())}",
            type="manipulation",
            parameters={
                "object_name": objects[0] if objects else "unknown_object",
                "action_type": "grasp",
                "grasp_type": "pinch"
            },
            priority=3,
            timeout=60
        ))

        # Return if needed
        if robot_context.get("return_to_origin", False):
            actions.append(Action(
                id=f"return_{int(datetime.datetime.now().timestamp())}",
                type="navigation",
                parameters={
                    "target_location": "origin",
                    "location_name": "starting position"
                },
                priority=4,
                timeout=120
            ))

        return actions

    def _sequence_navigate_and_report(self, parsed_goal: Dict[str, Any], robot_context: Dict[str, Any]) -> List[Action]:
        """
        Sequence for navigating and reporting
        """
        actions = []

        locations = parsed_goal.get("locations", ["unknown"])
        if locations:
            actions.append(Action(
                id=f"navigate_{locations[0]}_{int(datetime.datetime.now().timestamp())}",
                type="navigation",
                parameters={
                    "target_location": locations[0],
                    "location_name": locations[0]
                },
                priority=1,
                timeout=120
            ))

        actions.append(Action(
            id=f"report_{int(datetime.datetime.now().timestamp())}",
            type="communication",
            parameters={
                "message": f"Successfully reached {locations[0] if locations else 'destination'}",
                "action_type": "speak"
            },
            priority=2,
            timeout=10
        ))

        return actions

    def _sequence_clean_area(self, parsed_goal: Dict[str, Any], robot_context: Dict[str, Any]) -> List[Action]:
        """
        Sequence for cleaning an area
        """
        actions = []

        locations = parsed_goal.get("locations", ["unknown"])
        if locations:
            actions.append(Action(
                id=f"navigate_to_clean_{int(datetime.datetime.now().timestamp())}",
                type="navigation",
                parameters={
                    "target_location": locations[0],
                    "location_name": locations[0]
                },
                priority=1,
                timeout=120
            ))

        actions.append(Action(
            id=f"clean_{locations[0] if locations else 'area'}_{int(datetime.datetime.now().timestamp())}",
            type="manipulation",
            parameters={
                "action_type": "cleaning",
                "target_area": locations[0] if locations else "unknown_area",
                "cleaning_tool": "generic_cleaner"
            },
            priority=2,
            timeout=180
        ))

        return actions

    def _sequence_inspect_object(self, parsed_goal: Dict[str, Any], robot_context: Dict[str, Any]) -> List[Action]:
        """
        Sequence for inspecting an object
        """
        actions = []

        objects = parsed_goal.get("objects", ["unknown"])

        # Navigate close to object if location is known
        locations = parsed_goal.get("locations", [])
        if locations:
            actions.append(Action(
                id=f"navigate_to_inspect_{int(datetime.datetime.now().timestamp())}",
                type="navigation",
                parameters={
                    "target_location": locations[0],
                    "location_name": locations[0]
                },
                priority=1,
                timeout=120
            ))

        actions.append(Action(
            id=f"inspect_{objects[0] if objects else 'object'}_{int(datetime.datetime.now().timestamp())}",
            type="perception",
            parameters={
                "target_object": objects[0] if objects else "unknown_object",
                "action_type": "inspection",
                "inspection_type": "visual"
            },
            priority=2,
            timeout=60
        ))

        return actions

    def _default_sequence(self, parsed_goal: Dict[str, Any], robot_context: Dict[str, Any]) -> List[Action]:
        """
        Default sequence for unknown intents
        """
        return [
            Action(
                id=f"unknown_intent_{int(datetime.datetime.now().timestamp())}",
                type="communication",
                parameters={
                    "message": f"Unknown intent: {parsed_goal.get('intent', 'unknown')}",
                    "action_type": "speak"
                },
                priority=0,
                timeout=10
            )
        ]

    def demo_sequencing(self):
        """
        Demonstrate the task sequencing
        """
        print("Task Sequencer Demo")
        print("=" * 30)

        # Example parsed goals
        example_goals = [
            {
                "intent": "fetch_item",
                "objects": ["water", "glass"],
                "locations": ["kitchen"],
                "constraints": ["carefully"],
                "required_capabilities": ["navigation", "manipulation"]
            },
            {
                "intent": "navigate_and_report",
                "objects": [],
                "locations": ["living room"],
                "constraints": [],
                "required_capabilities": ["navigation", "communication"]
            }
        ]

        robot_context = {
            "robot_id": "robot_001",
            "return_to_origin": True
        }

        for i, goal in enumerate(example_goals, 1):
            print(f"\nExample {i}: {goal['intent']} with {goal['objects']} from {goal['locations']}")

            sequence = self.sequence_tasks(goal, robot_context)

            print(f"Generated sequence with {len(sequence.steps)} actions:")
            for j, action in enumerate(sequence.steps):
                print(f"  {j+1}. {action.type}: {action.parameters}")

if __name__ == "__main__":
    sequencer = TaskSequencer()
    sequencer.demo_sequencing()
```

### 6.3 LLM Reasoning System Example

```python
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
```

## 7. Assessment Questions for Chapter 2

1. Explain the challenges of natural language understanding in robotics applications.

2. Describe the different strategies for LLM-based task decomposition.

3. How do you convert LLM-generated plans to ROS 2 action sequences?

4. What are the key components of cognitive reasoning for robots?

5. Design a safety validation system for LLM-generated robot plans.

## Summary

In this chapter, we explored cognitive planning with Large Language Models for humanoid robots. We covered:

- Natural language goal parsing and understanding
- LLM-based task decomposition techniques
- Conversion of LLM plans to ROS 2 action sequences
- Cognitive reasoning capabilities with LLMs
- Safety and validation for LLM-generated plans

These concepts enable robots to understand high-level goals expressed in natural language and translate them into executable action sequences, making human-robot interaction more intuitive and natural.

## Next Steps

Continue to [Chapter 3: Autonomous Humanoid Capstone Project](./chapter-3-autonomous-capstone.md) to integrate these voice control and LLM planning systems into a complete autonomous humanoid robot system.

Go back to [Chapter 1: Voice Control and Speech Recognition](./chapter-1-voice-control.md) to review voice control concepts.

Or go back to the [Module 4 Overview](./index.md) to see all chapters.

## References and Further Reading

1. Wei, J., et al. (2022). "Emergent Abilities of Large Language Models." *Transactions on Machine Learning Research*.

2. Karpas, E., et al. (2022). "Plan, Attend, Generate: Planning for Sequential Text Generation." *arXiv preprint arXiv:2205.07836*.

3. Brohan, A., et al. (2022). "RT-1: Robotics Transformer for Real-World Control at Scale." *arXiv preprint arXiv:2202.01197*.

4. Huang, W., et al. (2022). "Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Agents." *International Conference on Machine Learning*.

5. Chen, H., et al. (2023). "Large Language Models for Robotics: Opportunities and Risks." *arXiv preprint arXiv:2307.03849*.