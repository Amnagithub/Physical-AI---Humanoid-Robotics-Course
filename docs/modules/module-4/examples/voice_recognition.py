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