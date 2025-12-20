# File: docs/modules/module-4/examples/voice_to_action.py

from utils import Action, ActionSequence, VoiceCommand
import json
from datetime import datetime

class VoiceToActionMapper:
    def __init__(self):
        # Define command mappings
        self.command_mappings = {
            "navigation": {
                "go to": self.map_navigation,
                "move to": self.map_navigation,
                "navigate to": self.map_navigation,
            },
            "manipulation": {
                "pick up": self.map_manipulation,
                "grasp": self.map_manipulation,
                "take": self.map_manipulation,
            },
            "perception": {
                "look at": self.map_perception,
                "find": self.map_perception,
                "locate": self.map_perception,
            }
        }

        # Location mappings
        self.locations = {
            "kitchen": {"x": 1.0, "y": 2.0, "z": 0.0},
            "living room": {"x": 3.0, "y": 4.0, "z": 0.0},
            "bedroom": {"x": 5.0, "y": 6.0, "z": 0.0},
            "office": {"x": 7.0, "y": 8.0, "z": 0.0}
        }

    def map_voice_command(self, command: VoiceCommand) -> ActionSequence:
        """
        Map a voice command to an action sequence
        """
        text = command.text.lower()

        # Find the appropriate mapping
        for action_type, mappings in self.command_mappings.items():
            for keyword, mapper_func in mappings.items():
                if keyword in text:
                    return mapper_func(text, command.id)

        # If no specific mapping found, return a default response
        return self.create_default_action(command.id)

    def map_navigation(self, text: str, command_id: str) -> ActionSequence:
        """
        Map navigation commands
        """
        # Extract location from text
        target_location = None
        for location_name in self.locations:
            if location_name in text:
                target_location = location_name
                break

        if target_location:
            location_data = self.locations[target_location]

            action = Action(
                id=f"nav_{command_id}",
                type="navigation",
                parameters={
                    "target_pose": location_data,
                    "location_name": target_location
                },
                priority=1,
                timeout=120
            )

            return ActionSequence(
                id=f"seq_{command_id}",
                steps=[action],
                status="pending",
                created_at=command.timestamp,
                robot_id="robot_001"
            )
        else:
            # If no specific location found, try to extract coordinates
            coordinates = self.extract_coordinates(text)
            if coordinates:
                action = Action(
                    id=f"nav_{command_id}",
                    type="navigation",
                    parameters={"target_pose": coordinates},
                    priority=1,
                    timeout=120
                )

                return ActionSequence(
                    id=f"seq_{command_id}",
                    steps=[action],
                    status="pending",
                    created_at=command.timestamp,
                    robot_id="robot_001"
                )

        return self.create_default_action(command_id)

    def map_manipulation(self, text: str, command_id: str) -> ActionSequence:
        """
        Map manipulation commands
        """
        # Extract object information
        object_name = self.extract_object_name(text)

        action = Action(
            id=f"manip_{command_id}",
            type="manipulation",
            parameters={
                "object_name": object_name,
                "action_type": "grasp",
                "grasp_type": "pinch"
            },
            priority=2,
            timeout=60
        )

        return ActionSequence(
            id=f"seq_{command_id}",
            steps=[action],
            status="pending",
            created_at=command.timestamp,
            robot_id="robot_001"
        )

    def map_perception(self, text: str, command_id: str) -> ActionSequence:
        """
        Map perception commands
        """
        # Extract object or target to look for
        target = self.extract_object_name(text) or self.extract_target(text)

        action = Action(
            id=f"percept_{command_id}",
            type="perception",
            parameters={
                "target_object": target,
                "action_type": "detection"
            },
            priority=1,
            timeout=30
        )

        return ActionSequence(
            id=f"seq_{command_id}",
            steps=[action],
            status="pending",
            created_at=command.timestamp,
            robot_id="robot_001"
        )

    def extract_object_name(self, text: str) -> str:
        """
        Extract object name from text
        """
        # Common objects that robots might interact with
        common_objects = [
            "ball", "cup", "book", "phone", "bottle",
            "box", "chair", "table", "door", "window"
        ]

        for obj in common_objects:
            if obj in text:
                return obj

        return "unknown_object"

    def extract_target(self, text: str) -> str:
        """
        Extract target from text
        """
        # Look for common targets
        if "red" in text:
            return "red_object"
        elif "blue" in text:
            return "blue_object"
        elif "big" in text:
            return "large_object"
        elif "small" in text:
            return "small_object"

        return "object"

    def extract_coordinates(self, text: str) -> dict:
        """
        Extract coordinates from text (simplified)
        """
        # Look for coordinate patterns like "x=1.0, y=2.0"
        # This is a simplified implementation
        import re

        # Pattern to match coordinates
        x_pattern = r"x[=\s]*([+-]?\d*\.?\d+)"
        y_pattern = r"y[=\s]*([+-]?\d*\.?\d+)"

        x_match = re.search(x_pattern, text)
        y_match = re.search(y_pattern, text)

        if x_match and y_match:
            return {
                "x": float(x_match.group(1)),
                "y": float(y_match.group(1)),
                "z": 0.0
            }

        return None

    def create_default_action(self, command_id: str) -> ActionSequence:
        """
        Create a default action when command cannot be mapped
        """
        action = Action(
            id=f"default_{command_id}",
            type="communication",
            parameters={
                "message": "I don't understand that command",
                "action_type": "speak"
            },
            priority=0,
            timeout=10
        )

        return ActionSequence(
            id=f"seq_{command_id}",
            steps=[action],
            status="pending",
            created_at=datetime.now(),
            robot_id="robot_001"
        )

    def demo_mapping(self):
        """
        Demonstrate the voice-to-action mapping
        """
        print("Voice-to-Action Mapping Demo")
        print("=" * 40)

        # Test commands
        test_commands = [
            "Go to the kitchen",
            "Move to the living room",
            "Pick up the red ball",
            "Look at the blue cup",
            "Navigate to x=5.0, y=3.0",
            "Take the book from the table"
        ]

        for i, cmd_text in enumerate(test_commands, 1):
            print(f"\nTest {i}: '{cmd_text}'")

            # Create a voice command object
            command = VoiceCommand(
                id=f"test_cmd_{i}",
                text=cmd_text,
                timestamp=datetime.now(),
                confidence=0.9,
                context={}
            )

            # Map the command
            action_seq = self.map_voice_command(command)

            print(f"Mapped to action sequence:")
            print(f"  ID: {action_seq.id}")
            print(f"  Status: {action_seq.status}")
            print(f"  Steps: {len(action_seq.steps)}")

            for j, action in enumerate(action_seq.steps):
                print(f"    Step {j+1}: {action.type}")
                print(f"      Parameters: {json.dumps(action.parameters, indent=6)}")

if __name__ == "__main__":
    mapper = VoiceToActionMapper()
    mapper.demo_mapping()