---
title: "Chapter 1: Voice Control and Speech Recognition"
sidebar_label: "Chapter 1: Voice Control"
---

# Chapter 1: Voice Control and Speech Recognition

## Overview

This chapter introduces Voice-to-Action systems for humanoid robots, focusing on speech recognition fundamentals and the integration of OpenAI Whisper with ROS 2. You'll learn how to convert voice commands into robot actions using speech and perception models.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the fundamentals of speech recognition for robotics
- Integrate OpenAI Whisper for voice command processing
- Design a voice command processing pipeline
- Create multimodal actions combining voice and vision
- Integrate voice commands with ROS 2 action servers

## 1. Speech Recognition Fundamentals

### 1.1 Introduction to Speech Recognition

Speech recognition is the technology that converts spoken language into text format. In robotics, this enables natural human-robot interaction through voice commands. The process involves several stages:

1. **Audio Capture**: Capturing sound waves using microphones
2. **Signal Processing**: Converting analog audio to digital format
3. **Feature Extraction**: Identifying speech patterns in the audio
4. **Recognition**: Matching features to known words and phrases
5. **Output**: Converting to text for further processing

### 1.2 Challenges in Robotic Speech Recognition

Robotic applications present unique challenges for speech recognition:

- **Environmental Noise**: Robots often operate in noisy environments
- **Distance and Direction**: Microphones may be far from speakers
- **Real-time Processing**: Need for low-latency responses
- **Vocabulary Constraints**: Limited to robot-appropriate commands
- **Robustness**: Must work across different speakers and accents

### 1.3 Evaluation Metrics

Key metrics for speech recognition systems include:

- **Word Error Rate (WER)**: Percentage of incorrectly recognized words
- **Latency**: Time from speech input to text output
- **Robustness**: Performance under various environmental conditions
- **Accuracy**: Correct recognition of intended commands

## 2. OpenAI Whisper Integration

### 2.1 Introduction to Whisper

OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) system trained on 680,000 hours of multilingual and multitask supervised data. It provides high accuracy across multiple languages and performs well even in challenging acoustic environments.

### 2.2 Whisper in Robotics Context

Whisper is particularly well-suited for robotics applications because:

- **High Accuracy**: Performs well on diverse audio inputs
- **Multilingual Support**: Handles multiple languages
- **Robustness**: Functions in noisy environments
- **API Access**: Easy integration via OpenAI API
- **Real-time Capabilities**: Can process streaming audio

### 2.3 Implementation Example

Here's how to integrate Whisper for robotic voice command processing:

```python
import openai
from utils import load_api_key, VoiceCommand
from datetime import datetime

class VoiceCommandProcessor:
    def __init__(self):
        api_key = load_api_key()
        if api_key:
            openai.api_key = api_key
        else:
            raise ValueError("OpenAI API key not found")

    def process_audio(self, audio_file_path):
        """
        Process audio file using OpenAI Whisper
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)

            command = VoiceCommand(
                id=f"cmd_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
                text=transcript.text,
                timestamp=datetime.now(),
                confidence=transcript.confidence if hasattr(transcript, 'confidence') else 0.9,
                context={}
            )

            return command
        except Exception as e:
            print(f"Error processing audio: {e}")
            return None

# Example usage
processor = VoiceCommandProcessor()
command = processor.process_audio("voice_command.wav")
if command:
    print(f"Recognized command: {command.text}")
    print(f"Confidence: {command.confidence}")
```

## 3. Voice Command Processing Pipeline

### 3.1 Pipeline Architecture

The voice command processing pipeline consists of several stages:

1. **Audio Input**: Capture from microphone or audio file
2. **Preprocessing**: Noise reduction and audio normalization
3. **Speech Recognition**: Convert speech to text using Whisper
4. **Natural Language Understanding**: Parse the intent from text
5. **Command Validation**: Ensure the command is appropriate and safe
6. **Action Mapping**: Convert to robot-appropriate actions

### 3.2 Preprocessing Considerations

Effective preprocessing can significantly improve recognition accuracy:

- **Noise Reduction**: Apply filters to reduce background noise
- **Audio Normalization**: Ensure consistent volume levels
- **Voice Activity Detection**: Identify when speech is present
- **Audio Format Conversion**: Ensure compatibility with Whisper

### 3.3 Implementation Example

```python
import pyaudio
import wave
import audioop
import threading
from queue import Queue
from utils import sanitize_text_input, validate_voice_command

class VoiceInputHandler:
    def __init__(self, chunk=1024, format=pyaudio.paInt16, channels=1, rate=44100):
        self.chunk = chunk
        self.format = format
        self.channels = channels
        self.rate = rate
        self.audio = pyaudio.PyAudio()
        self.is_recording = False
        self.audio_queue = Queue()

    def start_recording(self, duration=5):
        """
        Start recording audio for specified duration
        """
        self.is_recording = True
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        frames = []
        for _ in range(0, int(self.rate / self.chunk * duration)):
            if not self.is_recording:
                break
            data = stream.read(self.chunk)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        # Save to temporary file
        filename = f"temp_recording_{int(datetime.now().timestamp())}.wav"
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.audio.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return filename

    def stop_recording(self):
        """
        Stop the current recording
        """
        self.is_recording = False

class VoiceCommandPipeline:
    def __init__(self):
        self.input_handler = VoiceInputHandler()
        self.command_processor = VoiceCommandProcessor()

    def process_live_command(self):
        """
        Process a live voice command from microphone
        """
        print("Listening for voice command...")
        audio_file = self.input_handler.start_recording(duration=5)

        # Process the recorded audio
        command = self.command_processor.process_audio(audio_file)

        # Validate the command
        if command and validate_voice_command(command):
            # Sanitize the text input
            command.text = sanitize_text_input(command.text)
            print(f"Valid command received: {command.text}")
            return command
        else:
            print("Invalid or low-confidence command")
            return None
```

## 4. Multimodal Action Creation (Voice + Vision)

### 4.1 Introduction to Multimodal Systems

Multimodal systems combine multiple sensory inputs (voice, vision, touch) to create more robust and capable robotic systems. In the context of voice control, combining voice commands with visual information allows for more precise and context-aware robot behavior.

### 4.2 Visual Context Integration

When a robot receives a voice command, visual information can help disambiguate the intent:

- **Object Reference**: "Pick up the red ball" - identify the specific red ball
- **Location Context**: "Go to the kitchen" - use visual map to navigate
- **Action Constraints**: "Open the door" - identify which door to open
- **Safety Verification**: Ensure the requested action is safe given visual context

### 4.3 Implementation Example

```python
import cv2
import numpy as np
from utils import Action, ActionSequence

class MultimodalCommandProcessor:
    def __init__(self):
        self.voice_processor = VoiceCommandProcessor()
        self.vision_processor = VisionProcessor()  # Assume this exists

    def process_multimodal_command(self, audio_input, visual_input):
        """
        Process a command combining voice and visual inputs
        """
        # Process the voice command
        voice_command = self.voice_processor.process_audio(audio_input)
        if not voice_command:
            return None

        # Extract visual context
        visual_context = self.vision_processor.process_frame(visual_input)

        # Combine voice and vision for action generation
        action_sequence = self.create_multimodal_action(
            voice_command.text,
            visual_context
        )

        return action_sequence

    def create_multimodal_action(self, voice_text, visual_context):
        """
        Create an action sequence combining voice intent and visual context
        """
        # Example: "Pick up the red ball near the table"
        if "pick up" in voice_text.lower():
            # Find red objects in visual context
            red_objects = self.find_red_objects(visual_context)

            # Find the table in visual context
            table_location = self.find_table(visual_context)

            # Identify the red ball near the table
            target_object = self.find_closest_object(red_objects, table_location)

            if target_object:
                # Create a navigation and manipulation sequence
                action_sequence = ActionSequence(
                    id=f"multimodal_{int(datetime.now().timestamp())}",
                    steps=[
                        Action(
                            id="navigate_to_object",
                            type="navigation",
                            parameters={
                                "target_x": target_object['x'],
                                "target_y": target_object['y']
                            },
                            priority=1,
                            timeout=60
                        ),
                        Action(
                            id="grasp_object",
                            type="manipulation",
                            parameters={
                                "object_id": target_object['id'],
                                "grasp_type": "pinch"
                            },
                            priority=2,
                            timeout=30
                        )
                    ],
                    status="pending",
                    created_at=datetime.now(),
                    robot_id="robot_001"
                )
                return action_sequence

        # Fallback to voice-only processing
        return self.create_voice_only_action(voice_text)

    def find_red_objects(self, visual_context):
        """
        Find red objects in the visual context
        """
        # Implementation would use computer vision techniques
        # to identify red objects in the image
        pass

    def find_table(self, visual_context):
        """
        Find table in the visual context
        """
        # Implementation would use computer vision techniques
        # to identify tables in the image
        pass

    def find_closest_object(self, objects, reference_location):
        """
        Find the object closest to the reference location
        """
        # Implementation would calculate distances and return
        # the closest object to the reference location
        pass
```

## 5. ROS 2 Action Server Integration

### 5.1 ROS 2 Actions Overview

ROS 2 actions are a communication pattern for long-running tasks that require feedback and status updates. They are ideal for robot behaviors that:

- Take a significant amount of time to complete
- Need to provide feedback during execution
- Can be preempted or canceled
- Have clear success/failure outcomes

### 5.2 Action Message Structure

ROS 2 actions have three message types:

- **Goal**: The request to start an action
- **Feedback**: Information sent during execution
- **Result**: The final outcome of the action

### 5.3 Implementation Example

```python
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from example_interfaces.action import NavigateToPose  # Example action
from geometry_msgs.msg import Pose
from utils import format_action_for_ros2

class VoiceToActionBridge(Node):
    def __init__(self):
        super().__init__('voice_to_action_bridge')

        # Create action clients for different robot capabilities
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Service to receive voice commands
        self.voice_command_service = self.create_service(
            VoiceCommand,
            'process_voice_command',
            self.process_voice_command_callback
        )

    def process_voice_command_callback(self, request, response):
        """
        Process incoming voice command and convert to ROS 2 action
        """
        try:
            # Parse the voice command
            action_sequence = self.parse_voice_command(request.command)

            # Execute the action sequence
            success = self.execute_action_sequence(action_sequence)

            response.success = success
            response.message = "Command processed successfully" if success else "Command failed"

        except Exception as e:
            response.success = False
            response.message = f"Error processing command: {str(e)}"

        return response

    def parse_voice_command(self, command_text):
        """
        Parse voice command text and convert to action sequence
        """
        # This would contain the logic to convert natural language
        # to structured robot actions
        if "go to" in command_text.lower():
            # Extract target location from command
            target = self.extract_location(command_text)
            action = Action(
                id="navigation_action",
                type="navigation",
                parameters={"target_location": target},
                priority=1,
                timeout=120
            )
            return ActionSequence(
                id="nav_sequence",
                steps=[action],
                status="pending",
                created_at=datetime.now(),
                robot_id="robot_001"
            )

        # Add more command parsing logic here
        return None

    def execute_action_sequence(self, sequence):
        """
        Execute a sequence of ROS 2 actions
        """
        for action in sequence.steps:
            formatted_action = format_action_for_ros2(action)

            if action.type == "navigation":
                success = self.execute_navigation_action(formatted_action)
            elif action.type == "manipulation":
                success = self.execute_manipulation_action(formatted_action)
            else:
                self.get_logger().warn(f"Unknown action type: {action.type}")
                success = False

            if not success:
                return False  # Stop execution on failure

        return True

    def execute_navigation_action(self, action_data):
        """
        Execute a navigation action
        """
        # Wait for action server
        self.nav_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = Pose()  # Set target pose
        # Set pose parameters based on action_data

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        # Handle the response appropriately

        return True  # Simplified for example

    def extract_location(self, command_text):
        """
        Extract location information from voice command
        """
        # This would contain logic to extract location from
        # natural language commands like "go to the kitchen"
        locations = {
            "kitchen": {"x": 1.0, "y": 2.0},
            "living room": {"x": 3.0, "y": 4.0},
            "bedroom": {"x": 5.0, "y": 6.0}
        }

        for location_name, coordinates in locations.items():
            if location_name in command_text.lower():
                return coordinates

        return {"x": 0.0, "y": 0.0}  # Default location

def main(args=None):
    rclpy.init(args=args)

    voice_bridge = VoiceToActionBridge()

    try:
        rclpy.spin(voice_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        voice_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6. Practical Examples for Voice Command Recognition

### 6.1 Basic Voice Command Recognition

Let's create a complete example that demonstrates voice command recognition:

```python
# File: docs/modules/module-4/examples/voice_recognition.py

import openai
import pyaudio
import wave
import threading
from datetime import datetime
from utils import load_api_key, VoiceCommand, validate_voice_command

class VoiceRecognitionDemo:
    def __init__(self):
        # Load API key
        api_key = load_api_key()
        if api_key:
            openai.api_key = api_key
        else:
            raise ValueError("OpenAI API key not found")

        # Audio parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.audio = pyaudio.PyAudio()

    def record_audio(self, filename, duration=5):
        """
        Record audio from microphone and save to file
        """
        print(f"Recording for {duration} seconds...")

        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        frames = []
        for _ in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Recording finished.")

        stream.stop_stream()
        stream.close()

        # Save to file
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.audio.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

    def recognize_speech(self, audio_file_path):
        """
        Recognize speech using OpenAI Whisper
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)

            # Create VoiceCommand object
            command = VoiceCommand(
                id=f"demo_cmd_{int(datetime.now().timestamp())}",
                text=transcript.text,
                timestamp=datetime.now(),
                confidence=0.9,  # Whisper doesn't provide confidence directly
                context={"source": "microphone", "timestamp": datetime.now().isoformat()}
            )

            return command
        except Exception as e:
            print(f"Error in speech recognition: {e}")
            return None

    def run_demo(self):
        """
        Run the complete voice recognition demo
        """
        print("Voice Recognition Demo")
        print("=" * 30)

        # Record audio
        audio_filename = f"demo_audio_{int(datetime.now().timestamp())}.wav"
        self.record_audio(audio_filename, duration=5)

        # Recognize speech
        command = self.recognize_speech(audio_filename)

        if command:
            # Validate the command
            if validate_voice_command(command, min_confidence=0.7):
                print(f"\nRecognized Command: {command.text}")
                print(f"Confidence: {command.confidence}")
                print(f"Timestamp: {command.timestamp}")

                # Process the command (simple echo for demo)
                self.process_command(command)
            else:
                print("Command did not meet validation criteria")
        else:
            print("Failed to recognize speech")

    def process_command(self, command):
        """
        Process the recognized command
        """
        text = command.text.lower()

        if "hello" in text:
            print("Robot response: Hello! How can I help you?")
        elif "move" in text or "go" in text:
            print("Robot response: I can move to different locations")
        elif "stop" in text:
            print("Robot response: Stopping current action")
        else:
            print(f"Robot response: I received your command: '{command.text}'")

if __name__ == "__main__":
    demo = VoiceRecognitionDemo()
    demo.run_demo()
```

### 6.2 Voice-to-Action Mapping Example

```python
# File: docs/modules/module-4/examples/voice_to_action.py

from utils import Action, ActionSequence, VoiceCommand
import json

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
```

### 6.3 Voice-Controlled Robot Simulation Example

```python
# File: docs/modules/module-4/examples/voice_robot_control.py

import time
import threading
from utils import VoiceCommand, Action, ActionSequence, validate_action_sequence
from voice_recognition import VoiceRecognitionDemo
from voice_to_action import VoiceToActionMapper

class VoiceControlledRobotSimulator:
    def __init__(self):
        self.voice_recognizer = VoiceRecognitionDemo()
        self.action_mapper = VoiceToActionMapper()
        self.is_running = False
        self.robot_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.robot_orientation = 0.0  # in radians

    def start_voice_control(self):
        """
        Start the voice control interface
        """
        print("Starting Voice-Controlled Robot Simulator")
        print("Say 'quit' to exit")
        print("=" * 50)

        self.is_running = True

        while self.is_running:
            try:
                # Record and recognize voice command
                audio_filename = f"temp_{int(time.time())}.wav"
                self.voice_recognizer.record_audio(audio_filename, duration=3)

                command = self.voice_recognizer.recognize_speech(audio_filename)

                if command:
                    if validate_action_sequence(command, min_confidence=0.6):
                        print(f"Recognized: {command.text}")

                        # Check for quit command
                        if "quit" in command.text.lower():
                            print("Exiting voice control...")
                            self.is_running = False
                            break

                        # Map voice command to actions
                        action_sequence = self.action_mapper.map_voice_command(command)

                        if validate_action_sequence(action_sequence):
                            print(f"Executing action sequence: {action_sequence.id}")
                            self.execute_action_sequence(action_sequence)
                        else:
                            print("Invalid action sequence generated")
                    else:
                        print("Command confidence too low or invalid")
                else:
                    print("No command recognized, try again")

                time.sleep(1)  # Brief pause between commands

            except KeyboardInterrupt:
                print("\nInterrupted by user")
                self.is_running = False
                break
            except Exception as e:
                print(f"Error in voice control: {e}")
                time.sleep(1)

    def execute_action_sequence(self, sequence: ActionSequence):
        """
        Execute an action sequence in the simulation
        """
        print(f"Executing {len(sequence.steps)} actions...")

        for i, action in enumerate(sequence.steps):
            print(f"Step {i+1}/{len(sequence.steps)}: {action.type}")

            success = self.execute_action(action)

            if not success:
                print(f"Action {action.id} failed, stopping sequence")
                sequence.status = "failed"
                break

        if sequence.status != "failed":
            sequence.status = "completed"
            print("Action sequence completed successfully")

    def execute_action(self, action: Action) -> bool:
        """
        Execute a single action in the simulation
        """
        try:
            if action.type == "navigation":
                return self.execute_navigation(action)
            elif action.type == "manipulation":
                return self.execute_manipulation(action)
            elif action.type == "perception":
                return self.execute_perception(action)
            elif action.type == "communication":
                return self.execute_communication(action)
            else:
                print(f"Unknown action type: {action.type}")
                return False
        except Exception as e:
            print(f"Error executing action {action.id}: {e}")
            return False

    def execute_navigation(self, action: Action) -> bool:
        """
        Execute navigation action in simulation
        """
        target_pose = action.parameters.get("target_pose", {})
        location_name = action.parameters.get("location_name", "unknown")

        print(f"Navigating to {location_name}")
        print(f"Target: ({target_pose.get('x', 0)}, {target_pose.get('y', 0)})")

        # Simulate navigation by updating robot position
        self.robot_position["x"] = target_pose.get("x", self.robot_position["x"])
        self.robot_position["y"] = target_pose.get("y", self.robot_position["y"])

        # Simulate time for navigation
        time.sleep(2)  # Simulate 2 seconds for navigation

        print(f"Arrived at position: ({self.robot_position['x']}, {self.robot_position['y']})")
        return True

    def execute_manipulation(self, action: Action) -> bool:
        """
        Execute manipulation action in simulation
        """
        object_name = action.parameters.get("object_name", "unknown")
        action_type = action.parameters.get("action_type", "grasp")

        print(f"Performing {action_type} on {object_name}")

        # Simulate manipulation
        time.sleep(1.5)  # Simulate 1.5 seconds for manipulation

        print(f"Manipulation of {object_name} completed")
        return True

    def execute_perception(self, action: Action) -> bool:
        """
        Execute perception action in simulation
        """
        target_object = action.parameters.get("target_object", "unknown")

        print(f"Looking for {target_object}")

        # Simulate perception (in a real system, this would involve image processing)
        time.sleep(1)  # Simulate 1 second for perception

        # Simulate finding the object
        print(f"Found {target_object} in the environment")
        return True

    def execute_communication(self, action: Action) -> bool:
        """
        Execute communication action in simulation
        """
        message = action.parameters.get("message", "")
        action_type = action.parameters.get("action_type", "speak")

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
```

## 7. Assessment Questions for Chapter 1

1. What are the main stages of the speech recognition process in robotics?

2. Explain the advantages of using OpenAI Whisper for robotic voice command processing.

3. Describe how visual context can improve voice command interpretation in robotics.

4. What are the three message types in ROS 2 actions and their purposes?

5. Design a voice command processing pipeline for a humanoid robot that needs to operate in a noisy environment.

## Summary

In this chapter, we've covered the fundamentals of voice-to-action systems for humanoid robots. We explored:

- Speech recognition fundamentals and challenges in robotics
- Integration of OpenAI Whisper for voice processing
- Construction of a complete voice command processing pipeline
- Creation of multimodal systems combining voice and vision
- Integration with ROS 2 action servers for robot execution

These concepts form the foundation for creating robots that can understand and respond to natural voice commands, enabling more intuitive human-robot interaction.

## Next Steps

Continue to [Chapter 2: Cognitive Planning with Large Language Models](./chapter-2-llm-planning.md) to learn how to enhance these systems with cognitive planning using LLMs.

Or go back to the [Module 4 Overview](./index.md) to see all chapters.

## References and Further Reading

1. Radfar, A., et al. (2023). "Advances in Speech Recognition for Robotics." *IEEE Robotics & Automation Magazine*.

2. OpenAI. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." *arXiv preprint arXiv:2212.04356*.

3. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. (Chapter on Audio-based Perception)

4. ROS 2 Documentation. "Working with Audio and Speech." *docs.ros.org*

5. Brown, T., et al. (2020). "Language Models are Few-Shot Learners." *Advances in Neural Information Processing Systems*, 33, 1877-1901.