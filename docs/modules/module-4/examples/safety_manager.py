# File: docs/modules/module-4/examples/safety_manager.py

"""
Safety Manager and Error Recovery Manager for VLA Capstone Project
"""

import time
from datetime import datetime
from enum import Enum
from typing import List, Dict, Any
from utils import Action

class SafetyLevel(Enum):
    SAFE = "safe"
    WARNING = "warning"
    DANGEROUS = "dangerous"
    EMERGENCY = "emergency"

class SafetyManager:
    """
    Manages safety checks and emergency procedures for the VLA system
    """
    def __init__(self):
        self.safety_level = SafetyLevel.SAFE
        self.emergency_stop = False
        self.safety_log = []
        self.max_retries = 3

    def check_safety_before_action(self, action: Action, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Check if an action is safe to execute
        """
        issues = []

        # Check action type safety
        if action.type == "navigation":
            issues.extend(self._check_navigation_safety(action, context))
        elif action.type == "manipulation":
            issues.extend(self._check_manipulation_safety(action, context))

        # Check environmental safety
        issues.extend(self._check_environmental_safety(context))

        # Check robot capability safety
        issues.extend(self._check_capability_safety(action, context))

        is_safe = len(issues) == 0
        safety_level = SafetyLevel.DANGEROUS if issues else SafetyLevel.SAFE

        result = {
            "is_safe": is_safe,
            "safety_level": safety_level,
            "issues": issues,
            "action_id": action.id
        }

        self._log_safety_check(result)
        return result

    def _check_navigation_safety(self, action: Action, context: Dict[str, Any]) -> List[str]:
        """
        Check navigation-specific safety issues
        """
        issues = []

        target_pose = action.parameters.get("target_pose", {})
        forbidden_areas = context.get("forbidden_areas", [])

        # Check if target is in forbidden area
        for area in forbidden_areas:
            if self._is_in_forbidden_area(target_pose, area):
                issues.append(f"Navigation target {target_pose} is in forbidden area")

        # Check distance limits
        max_range = context.get("robot_max_range", 15.0)
        distance = self._calculate_distance(target_pose)
        if distance > max_range:
            issues.append(f"Navigation target too far: {distance:.2f}m (max: {max_range}m)")

        return issues

    def _check_manipulation_safety(self, action: Action, context: Dict[str, Any]) -> List[str]:
        """
        Check manipulation-specific safety issues
        """
        issues = []

        object_name = action.parameters.get("object_name", "").lower()

        # Check for dangerous objects
        dangerous_objects = ["knife", "blade", "sharp", "fire", "hot"]
        for dangerous in dangerous_objects:
            if dangerous in object_name:
                issues.append(f"Manipulation of potentially dangerous object: {object_name}")

        return issues

    def _check_environmental_safety(self, context: Dict[str, Any]) -> List[str]:
        """
        Check environmental safety factors
        """
        issues = []

        # Check lighting conditions
        lighting = context.get("lighting", "good")
        if lighting == "poor":
            issues.append("Poor lighting conditions, extra caution required")

        # Check for people in area
        people_in_area = context.get("people_in_area", True)
        if not people_in_area:
            issues.append("No people detected in area, verify robot should operate autonomously")

        return issues

    def _check_capability_safety(self, action: Action, context: Dict[str, Any]) -> List[str]:
        """
        Check if robot has capability to safely perform action
        """
        issues = []

        robot_capabilities = context.get("robot_capabilities", [])
        if action.type not in robot_capabilities:
            issues.append(f"Robot lacks capability for action: {action.type}")

        return issues

    def _is_in_forbidden_area(self, pose: Dict[str, float], area: Dict[str, float]) -> bool:
        """
        Check if a pose is within a forbidden area
        """
        if "min_x" in area and "max_x" in area and "min_y" in area and "max_y" in area:
            x = pose.get("x", 0)
            y = pose.get("y", 0)
            return (area["min_x"] <= x <= area["max_x"] and
                    area["min_y"] <= y <= area["max_y"])
        return False

    def _calculate_distance(self, pose: Dict[str, float]) -> float:
        """
        Calculate distance from origin to pose
        """
        x = pose.get("x", 0)
        y = pose.get("y", 0)
        return (x**2 + y**2)**0.5

    def _log_safety_check(self, result: Dict[str, Any]):
        """
        Log safety check results
        """
        self.safety_log.append({
            "timestamp": datetime.now().isoformat(),
            "action_id": result["action_id"],
            "is_safe": result["is_safe"],
            "safety_level": result["safety_level"].value,
            "issues": result["issues"]
        })

    def trigger_emergency_stop(self, reason: str):
        """
        Trigger emergency stop
        """
        self.emergency_stop = True
        self.safety_level = SafetyLevel.EMERGENCY
        self._log_safety_event("EMERGENCY_STOP", reason)

    def _log_safety_event(self, event_type: str, reason: str):
        """
        Log a safety event
        """
        self.safety_log.append({
            "timestamp": datetime.now().isoformat(),
            "event_type": event_type,
            "reason": reason
        })


class ErrorRecoveryManager:
    """
    Manages error recovery strategies for the VLA system
    """
    def __init__(self):
        self.recovery_strategies = {
            "navigation_failure": self._recover_navigation_failure,
            "manipulation_failure": self._recover_manipulation_failure,
            "communication_failure": self._recover_communication_failure,
            "perception_failure": self._recover_perception_failure
        }

    def attempt_recovery(self, error_type: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Attempt to recover from an error
        """
        if error_type in self.recovery_strategies:
            return self.recovery_strategies[error_type](context)
        else:
            return {
                "recovery_attempted": False,
                "new_action": None,
                "message": f"No recovery strategy for error type: {error_type}"
            }

    def _recover_navigation_failure(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Recovery strategy for navigation failures
        """
        # Try alternative route or return to safe position
        return {
            "recovery_attempted": True,
            "new_action": {
                "type": "navigation",
                "parameters": {"target_location": "safe_position"},
                "description": "Return to safe position after navigation failure"
            },
            "message": "Attempting to return to safe position"
        }

    def _recover_manipulation_failure(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Recovery strategy for manipulation failures
        """
        # Try alternative grasp or report failure
        return {
            "recovery_attempted": True,
            "new_action": {
                "type": "communication",
                "parameters": {"message": "Unable to grasp object, need assistance"},
                "description": "Report manipulation failure"
            },
            "message": "Reporting manipulation failure to user"
        }

    def _recover_communication_failure(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Recovery strategy for communication failures
        """
        # Retry or use alternative communication method
        return {
            "recovery_attempted": True,
            "new_action": {
                "type": "communication",
                "parameters": {"message": "Communication restored", "retry_count": 1},
                "description": "Confirm communication recovery"
            },
            "message": "Communication system recovered"
        }

    def _recover_perception_failure(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Recovery strategy for perception failures
        """
        # Try different sensing approach or report failure
        return {
            "recovery_attempted": True,
            "new_action": {
                "type": "communication",
                "parameters": {"message": "Unable to detect requested object, changing position"},
                "description": "Report perception failure and adjust strategy"
            },
            "message": "Adjusting approach due to perception failure"
        }