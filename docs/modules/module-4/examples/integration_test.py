# File: docs/modules/module-4/examples/integration_test.py

"""
Integration Tests for Vision-Language-Action (VLA) System
These tests verify that all components work together correctly
"""

import unittest
import asyncio
from datetime import datetime
from unittest.mock import Mock, patch
from utils import VoiceCommand, ActionSequence, Action, LLMPlan

# Import the components to test
from voice_recognition import VoiceRecognitionDemo
from voice_to_action import VoiceToActionMapper
from llm_goal_parser import LLMGoalParser
from task_sequencer import TaskSequencer
from llm_reasoning import LLMReasoningSystem
from safety_validator import SafetyValidator
from explanation_engine import ExplanationEngine, MonitoringSystem

class TestVLAIntegration(unittest.TestCase):
    """
    Integration tests for the complete VLA system
    """

    def setUp(self):
        """
        Set up test fixtures before each test method
        """
        # Mock components that require external services (like OpenAI API)
        self.mock_voice_recognizer = Mock(spec=VoiceRecognitionDemo)
        self.voice_mapper = VoiceToActionMapper()
        self.mock_llm_parser = Mock(spec=LLMGoalParser)
        self.task_sequencer = TaskSequencer()
        self.mock_reasoning_system = Mock(spec=LLMReasoningSystem)
        self.safety_validator = SafetyValidator()
        self.explanation_engine = ExplanationEngine()
        self.monitoring_system = MonitoringSystem()

        # Robot context for testing
        self.test_context = {
            "robot_id": "test_robot_001",
            "robot_capabilities": ["navigation", "manipulation", "perception", "communication"],
            "environment": "test environment",
            "forbidden_areas": [],
            "robot_max_range": 15.0
        }

    def test_voice_to_action_integration(self):
        """
        Test integration between voice recognition and action mapping
        """
        print("Testing Voice-to-Action Integration...")

        # Simulate a parsed goal from LLM (since we're mocking the LLM component)
        test_parsed_goal = {
            "intent": "fetch_item",
            "objects": ["water", "glass"],
            "locations": ["kitchen"],
            "constraints": ["carefully"],
            "required_capabilities": ["navigation", "manipulation"]
        }

        # Test task sequencing
        action_sequence = self.task_sequencer.sequence_tasks(test_parsed_goal, self.test_context)

        # Verify the sequence has the expected number of steps
        self.assertGreater(len(action_sequence.steps), 0, "Action sequence should have steps")

        # Verify specific action types are present
        action_types = [action.type for action in action_sequence.steps]
        self.assertIn("navigation", action_types, "Navigation action should be in sequence")
        self.assertIn("manipulation", action_types, "Manipulation action should be in sequence")

        print(f"  Generated sequence with {len(action_sequence.steps)} actions")
        for i, action in enumerate(action_sequence.steps):
            print(f"    {i+1}. {action.type}: {action.parameters}")

    def test_safety_validation_integration(self):
        """
        Test integration of safety validation with action sequences
        """
        print("Testing Safety Validation Integration...")

        # Create a test action sequence
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
            robot_id="test_robot_001"
        )

        # Validate the sequence
        validation_result = self.safety_validator.validate_plan(test_sequence, self.test_context)

        # Verify validation completed without errors
        self.assertIsInstance(validation_result, dict, "Validation result should be a dictionary")
        self.assertIn("is_safe", validation_result, "Validation result should have 'is_safe' field")
        self.assertIn("issues", validation_result, "Validation result should have 'issues' field")

        print(f"  Safety validation result: is_safe={validation_result['is_safe']}")
        print(f"  Issues found: {len(validation_result['issues'])}")

    def test_complete_vla_pipeline(self):
        """
        Test the complete VLA pipeline: goal parsing -> task sequencing -> safety validation
        """
        print("Testing Complete VLA Pipeline...")

        # Simulated parsed goal (in a real system, this would come from LLM)
        test_goal = {
            "intent": "navigate_and_report",
            "objects": [],
            "locations": ["living room"],
            "constraints": [],
            "required_capabilities": ["navigation", "communication"]
        }

        # Step 1: Task sequencing
        action_sequence = self.task_sequencer.sequence_tasks(test_goal, self.test_context)
        self.assertGreater(len(action_sequence.steps), 0, "Task sequencing should produce actions")

        # Step 2: Safety validation
        validation_result = self.safety_validator.validate_plan(action_sequence, self.test_context)
        self.assertIsInstance(validation_result, dict, "Validation should return a result dictionary")

        # Step 3: Generate explanation
        if action_sequence.steps:
            explanation = self.explanation_engine.generate_explanation(
                action_sequence.steps[0],
                ["User requested to go to living room", "Validated navigation safety"],
                0.9,
                self.test_context
            )
            self.assertIsNotNone(explanation, "Explanation should be generated")

        print(f"  Pipeline completed: {len(action_sequence.steps)} actions, safe={validation_result['is_safe']}")

    def test_explanation_generation_integration(self):
        """
        Test integration of explanation generation with other components
        """
        print("Testing Explanation Generation Integration...")

        # Create a sample action
        test_action = Action(
            id="test_explanation_action",
            type="navigation",
            parameters={"target_location": "kitchen"},
            priority=1,
            timeout=120
        )

        # Generate explanation
        reasoning_trace = [
            "Goal was to navigate to kitchen",
            "Kitchen is a valid destination",
            "Path is clear and safe"
        ]
        explanation = self.explanation_engine.generate_explanation(
            test_action,
            reasoning_trace,
            0.85,
            self.test_context
        )

        # Verify explanation was generated
        self.assertIsNotNone(explanation, "Explanation should be generated")
        self.assertIn("explanation", explanation, "Explanation should contain explanation text")
        self.assertIn("action_id", explanation, "Explanation should contain action ID")

        print(f"  Explanation generated for action: {explanation['action_id']}")
        print(f"  Natural language explanation: {explanation['explanation'][:100]}...")

    def test_monitoring_integration(self):
        """
        Test integration of monitoring with action execution simulation
        """
        print("Testing Monitoring Integration...")

        # Create and execute a simulated action
        test_action = Action(
            id="monitor_test_action",
            type="communication",
            parameters={"message": "Test message"},
            priority=0,
            timeout=10
        )

        # Simulate execution timing
        start_time = datetime.now()
        # Simulate some processing time
        import time
        time.sleep(0.01)  # Small delay to simulate processing
        end_time = datetime.now()

        # Log the action
        result = {"success": True, "message": "Action completed successfully"}
        self.monitoring_system.log_action(test_action, result, start_time, end_time)

        # Verify action was logged
        performance_report = self.monitoring_system.get_performance_report()
        self.assertGreater(performance_report["performance_metrics"]["total_actions"], 0,
                         "Action should be logged in monitoring system")

        print(f"  Actions logged: {performance_report['performance_metrics']['total_actions']}")
        print(f"  Success rate: {performance_report['success_rate']:.1f}%")

    def test_error_handling_integration(self):
        """
        Test how components handle errors when integrated
        """
        print("Testing Error Handling Integration...")

        # Test with an invalid action sequence
        invalid_sequence = ActionSequence(
            id="invalid_seq",
            steps=[],
            status="pending",
            created_at=datetime.now(),
            robot_id="test_robot_001"
        )

        # This should handle the empty sequence gracefully
        validation_result = self.safety_validator.validate_plan(invalid_sequence, self.test_context)
        self.assertIsInstance(validation_result, dict, "Should return a validation result even for invalid input")

        # Test with null/None inputs where appropriate
        try:
            # This should not crash the system
            empty_result = self.task_sequencer.sequence_tasks({}, self.test_context)
            print("  Handled empty goal gracefully")
        except Exception as e:
            print(f"  Error handling working as expected: {type(e).__name__}")

class TestVLAEndToEnd(unittest.TestCase):
    """
    End-to-end integration tests simulating real usage scenarios
    """

    def setUp(self):
        """
        Set up for end-to-end tests
        """
        # For end-to-end tests, we'll use partial mocking to test real integration
        # where possible while avoiding external dependencies
        self.task_sequencer = TaskSequencer()
        self.safety_validator = SafetyValidator()
        self.explanation_engine = ExplanationEngine()

        self.test_context = {
            "robot_id": "e2e_test_robot_001",
            "robot_capabilities": ["navigation", "manipulation", "perception", "communication"],
            "environment": "home environment",
            "forbidden_areas": [{"min_x": 10, "max_x": 20, "min_y": 10, "max_y": 20}],
            "robot_max_range": 15.0
        }

    def test_fetch_item_scenario(self):
        """
        End-to-end test: Fetch item scenario
        """
        print("\nE2E Test: Fetch Item Scenario")
        print("-" * 30)

        # Simulated goal from LLM parsing
        goal = {
            "intent": "fetch_item",
            "objects": ["water", "glass"],
            "locations": ["kitchen"],
            "constraints": ["carefully"],
            "required_capabilities": ["navigation", "manipulation"]
        }

        # 1. Generate action sequence
        sequence = self.task_sequencer.sequence_tasks(goal, self.test_context)
        print(f"1. Generated {len(sequence.steps)} action steps")

        # 2. Validate safety
        validation = self.safety_validator.validate_plan(sequence, self.test_context)
        print(f"2. Safety validation: {'PASS' if validation['is_safe'] else 'FAIL'}")

        # 3. Generate explanation for first action
        if sequence.steps:
            explanation = self.explanation_engine.generate_explanation(
                sequence.steps[0],
                ["User wants water", "Need to navigate to kitchen first"],
                0.9,
                self.test_context
            )
            print(f"3. Explanation generated: {explanation['explanation'][:80]}...")

        # 4. Verify expected action types are present
        action_types = [action.type for action in sequence.steps]
        expected_types = ["navigation", "manipulation"]
        for expected_type in expected_types:
            self.assertIn(expected_type, action_types,
                         f"Expected {expected_type} action in sequence")

        print("✓ Fetch item scenario completed successfully")

    def test_navigation_scenario(self):
        """
        End-to-end test: Navigation scenario
        """
        print("\nE2E Test: Navigation Scenario")
        print("-" * 30)

        # Simulated goal from LLM parsing
        goal = {
            "intent": "navigate_and_report",
            "objects": [],
            "locations": ["living room"],
            "constraints": [],
            "required_capabilities": ["navigation", "communication"]
        }

        # Generate and validate sequence
        sequence = self.task_sequencer.sequence_tasks(goal, self.test_context)
        validation = self.safety_validator.validate_plan(sequence, self.test_context)

        print(f"1. Generated {len(sequence.steps)} action steps")
        print(f"2. Safety validation: {'PASS' if validation['is_safe'] else 'FAIL'}")

        # Verify navigation and communication actions are present
        action_types = [action.type for action in sequence.steps]
        self.assertIn("navigation", action_types, "Should have navigation action")
        self.assertIn("communication", action_types, "Should have communication action")

        print("✓ Navigation scenario completed successfully")

def run_integration_tests():
    """
    Run all integration tests
    """
    print("="*60)
    print("VLA SYSTEM INTEGRATION TESTS")
    print("="*60)

    # Create test suite
    suite = unittest.TestSuite()

    # Add integration tests
    suite.addTest(unittest.makeSuite(TestVLAIntegration))
    suite.addTest(unittest.makeSuite(TestVLAEndToEnd))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print("\n" + "="*60)
    print("INTEGRATION TEST SUMMARY")
    print("="*60)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%" if result.testsRun > 0 else "0%")

    if result.failures:
        print("\nFailures:")
        for test, trace in result.failures:
            print(f"  {test}: {trace}")

    if result.errors:
        print("\nErrors:")
        for test, trace in result.errors:
            print(f"  {test}: {trace}")

    return result.wasSuccessful()

def main():
    """
    Main function to run integration tests
    """
    print("Running VLA System Integration Tests...")
    success = run_integration_tests()

    if success:
        print("\n✓ All integration tests passed!")
        print("The VLA system components integrate correctly.")
    else:
        print("\n✗ Some integration tests failed.")
        print("Review the failures and fix the integration issues.")

if __name__ == "__main__":
    main()