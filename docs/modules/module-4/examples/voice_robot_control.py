# File: docs/modules/module-4/examples/voice_robot_control.py

import time
import threading
from utils import VoiceCommand, Action, ActionSequence, validate_voice_command
from datetime import datetime
import json

class VoiceControlledRobotSimulator:
    def __init__(self):
        self.is_running = False
        self.robot_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.robot_orientation = 0.0  # in radians

    def simulate_voice_input(self, command_text):
        """
        Simulate voice input for demonstration purposes
        """
        command = VoiceCommand(
            id=f"sim_cmd_{int(time.time())}",
            text=command_text,
            timestamp=datetime.now(),
            confidence=0.85,
            context={"source": "simulation", "timestamp": datetime.now().isoformat()}
        )
        return command

    def start_voice_control(self):
        """
        Start the voice control interface (simulated)
        """
        print("Starting Voice-Controlled Robot Simulator")
        print("Commands: 'go to kitchen', 'pick up red ball', 'look at blue cup', 'quit'")
        print("=" * 50)

        self.is_running = True

        while self.is_running:
            try:
                # Get simulated command
                cmd_input = input("\nEnter voice command (or 'quit'): ").strip()

                if cmd_input.lower() == 'quit':
                    print("Exiting voice control...")
                    self.is_running = False
                    break

                # Simulate voice command
                command = self.simulate_voice_input(cmd_input)

                if validate_voice_command(command, min_confidence=0.6):
                    print(f"Recognized: {command.text}")

                    # Map voice command to actions
                    action_sequence = self.map_voice_command(command.text)

                    if self.validate_action_sequence(action_sequence):
                        print(f"Executing action sequence: {action_sequence['id']}")
                        self.execute_action_sequence(action_sequence)
                    else:
                        print("Invalid action sequence generated")
                else:
                    print("Command confidence too low or invalid")

            except KeyboardInterrupt:
                print("\nInterrupted by user")
                self.is_running = False
                break
            except Exception as e:
                print(f"Error in voice control: {e}")
                time.sleep(1)

    def map_voice_command(self, text: str) -> dict:
        """
        Map voice command text to action sequence (simplified)
        """
        text_lower = text.lower()
        actions = []

        if "go to" in text_lower or "move to" in text_lower or "navigate to" in text_lower:
            # Extract location
            location = "unknown"
            locations = ["kitchen", "living room", "bedroom", "office"]
            for loc in locations:
                if loc in text_lower:
                    location = loc
                    break

            actions.append({
                "id": f"nav_{int(time.time())}",
                "type": "navigation",
                "parameters": {
                    "target_location": location
                },
                "priority": 1,
                "timeout": 120
            })

        elif "pick up" in text_lower or "grasp" in text_lower or "take" in text_lower:
            # Extract object
            object_name = "unknown_object"
            objects = ["ball", "cup", "book", "phone", "bottle"]
            for obj in objects:
                if obj in text_lower:
                    object_name = obj
                    break

            actions.append({
                "id": f"manip_{int(time.time())}",
                "type": "manipulation",
                "parameters": {
                    "object_name": object_name,
                    "action_type": "grasp"
                },
                "priority": 2,
                "timeout": 60
            })

        elif "look at" in text_lower or "find" in text_lower or "locate" in text_lower:
            # Extract target
            target = "unknown_object"
            objects = ["ball", "cup", "book", "phone", "bottle"]
            colors = ["red", "blue", "green", "yellow"]

            for obj in objects:
                if obj in text_lower:
                    target = obj
                    break

            for color in colors:
                if color in text_lower:
                    target = f"{color} {target}"
                    break

            actions.append({
                "id": f"percept_{int(time.time())}",
                "type": "perception",
                "parameters": {
                    "target_object": target,
                    "action_type": "detection"
                },
                "priority": 1,
                "timeout": 30
            })

        else:
            # Default communication action
            actions.append({
                "id": f"comm_{int(time.time())}",
                "type": "communication",
                "parameters": {
                    "message": f"I received your command: {text}",
                    "action_type": "speak"
                },
                "priority": 0,
                "timeout": 10
            })

        return {
            "id": f"seq_{int(time.time())}",
            "steps": actions,
            "status": "pending",
            "created_at": datetime.now().isoformat(),
            "robot_id": "robot_001"
        }

    def validate_action_sequence(self, sequence: dict) -> bool:
        """
        Validate an action sequence
        """
        if not sequence.get("id"):
            return False

        steps = sequence.get("steps", [])
        if not steps:
            return False

        for action in steps:
            if not action.get("type") or not action.get("parameters"):
                return False

        valid_statuses = ["pending", "executing", "completed", "failed"]
        if sequence.get("status") not in valid_statuses:
            return False

        return True

    def execute_action_sequence(self, sequence: dict):
        """
        Execute an action sequence in the simulation
        """
        print(f"Executing {len(sequence['steps'])} actions...")

        for i, action in enumerate(sequence['steps']):
            print(f"Step {i+1}/{len(sequence['steps'])}: {action['type']}")

            success = self.execute_action(action)

            if not success:
                print(f"Action {action['id']} failed, stopping sequence")
                sequence["status"] = "failed"
                break

        if sequence["status"] != "failed":
            sequence["status"] = "completed"
            print("Action sequence completed successfully")

    def execute_action(self, action: dict) -> bool:
        """
        Execute a single action in the simulation
        """
        try:
            if action['type'] == "navigation":
                return self.execute_navigation(action)
            elif action['type'] == "manipulation":
                return self.execute_manipulation(action)
            elif action['type'] == "perception":
                return self.execute_perception(action)
            elif action['type'] == "communication":
                return self.execute_communication(action)
            else:
                print(f"Unknown action type: {action['type']}")
                return False
        except Exception as e:
            print(f"Error executing action {action['id']}: {e}")
            return False

    def execute_navigation(self, action: dict) -> bool:
        """
        Execute navigation action in simulation
        """
        target_location = action['parameters'].get("target_location", "unknown")

        print(f"Navigating to {target_location}")

        # Simulate navigation by updating robot position
        location_coords = {
            "kitchen": {"x": 1.0, "y": 2.0},
            "living room": {"x": 3.0, "y": 4.0},
            "bedroom": {"x": 5.0, "y": 6.0},
            "office": {"x": 7.0, "y": 8.0}
        }

        if target_location in location_coords:
            self.robot_position["x"] = location_coords[target_location]["x"]
            self.robot_position["y"] = location_coords[target_location]["y"]

        # Simulate time for navigation
        time.sleep(2)  # Simulate 2 seconds for navigation

        print(f"Arrived at {target_location}: ({self.robot_position['x']}, {self.robot_position['y']})")
        return True

    def execute_manipulation(self, action: dict) -> bool:
        """
        Execute manipulation action in simulation
        """
        object_name = action['parameters'].get("object_name", "unknown")
        action_type = action['parameters'].get("action_type", "grasp")

        print(f"Performing {action_type} on {object_name}")

        # Simulate manipulation
        time.sleep(1.5)  # Simulate 1.5 seconds for manipulation

        print(f"Manipulation of {object_name} completed")
        return True

    def execute_perception(self, action: dict) -> bool:
        """
        Execute perception action in simulation
        """
        target_object = action['parameters'].get("target_object", "unknown")

        print(f"Looking for {target_object}")

        # Simulate perception (in a real system, this would involve image processing)
        time.sleep(1)  # Simulate 1 second for perception

        # Simulate finding the object
        print(f"Found {target_object} in the environment")
        return True

    def execute_communication(self, action: dict) -> bool:
        """
        Execute communication action in simulation
        """
        message = action['parameters'].get("message", "")
        action_type = action['parameters'].get("action_type", "speak")

        print(f"Robot says: {message}")

        # Simulate communication
        time.sleep(0.5)
        return True

    def get_robot_status(self):
        """
        Get current robot status
        """
        return {
            "position": self.robot_position,
            "orientation": self.robot_orientation,
            "status": "active" if self.is_running else "inactive"
        }

def main():
    """
    Main function to run the voice-controlled robot simulator
    """
    simulator = VoiceControlledRobotSimulator()

    try:
        simulator.start_voice_control()
    except Exception as e:
        print(f"Error running simulator: {e}")
    finally:
        print("\nRobot simulator stopped.")
        print(f"Final robot position: {simulator.robot_position}")

if __name__ == "__main__":
    main()