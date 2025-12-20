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