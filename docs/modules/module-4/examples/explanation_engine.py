# File: docs/modules/module-4/examples/explanation_engine.py

"""
Explanation Engine and Monitoring System for VLA Capstone Project
"""

import json
from datetime import datetime
from typing import Dict, Any, List
from utils import Action

class ExplanationEngine:
    """
    Generates explanations for robot actions and decisions
    """
    def __init__(self):
        self.explanation_history = []

    def generate_explanation(self, action: Action, reasoning_trace: List[str],
                           confidence: float, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate an explanation for an action
        """
        explanation = {
            "action_id": action.id,
            "action_type": action.type,
            "action_parameters": action.parameters,
            "timestamp": datetime.now().isoformat(),
            "reasoning_trace": reasoning_trace,
            "confidence": confidence,
            "context": context,
            "explanation": self._create_natural_language_explanation(action, reasoning_trace, confidence)
        }

        self.explanation_history.append(explanation)
        return explanation

    def _create_natural_language_explanation(self, action: Action, reasoning_trace: List[str],
                                           confidence: float) -> str:
        """
        Create a natural language explanation of the action
        """
        action_type_descriptions = {
            "navigation": "move to a location",
            "manipulation": "interact with an object",
            "perception": "observe or detect something",
            "communication": "communicate with a person"
        }

        action_desc = action_type_descriptions.get(action.type, "perform an action")
        confidence_desc = "high" if confidence > 0.8 else "medium" if confidence > 0.5 else "low"

        explanation = f"I am going to {action_desc} because "

        if reasoning_trace:
            explanation += f"{reasoning_trace[0].lower()}. "
        else:
            explanation += "it was requested. "

        explanation += f"My confidence in this decision is {confidence_desc}."

        return explanation

    def get_explanation_for_action(self, action_id: str) -> Dict[str, Any]:
        """
        Get the explanation for a specific action
        """
        for explanation in self.explanation_history:
            if explanation["action_id"] == action_id:
                return explanation
        return None


class MonitoringSystem:
    """
    Monitors system performance and logs actions
    """
    def __init__(self):
        self.action_log = []
        self.performance_metrics = {
            "total_actions": 0,
            "successful_actions": 0,
            "failed_actions": 0,
            "average_execution_time": 0.0,
            "safety_violations": 0
        }
        self.system_status = {
            "uptime": 0,
            "current_load": 0,
            "last_error": None
        }

    def log_action(self, action: Action, result: Dict[str, Any], start_time: datetime,
                   end_time: datetime):
        """
        Log an action and its result
        """
        execution_time = (end_time - start_time).total_seconds()

        log_entry = {
            "action_id": action.id if action else None,
            "action_type": action.type if action else None,
            "parameters": action.parameters if action else {},
            "result": result,
            "execution_time": execution_time,
            "timestamp": start_time.isoformat(),
            "end_time": end_time.isoformat()
        }

        self.action_log.append(log_entry)

        # Update performance metrics
        self._update_performance_metrics(result, execution_time)

    def _update_performance_metrics(self, result: Dict[str, Any], execution_time: float):
        """
        Update performance metrics based on action result
        """
        self.performance_metrics["total_actions"] += 1

        if result.get("success", False):
            self.performance_metrics["successful_actions"] += 1
        else:
            self.performance_metrics["failed_actions"] += 1

        # Update average execution time
        total_time = self.performance_metrics["average_execution_time"] * (self.performance_metrics["total_actions"] - 1)
        total_time += execution_time
        self.performance_metrics["average_execution_time"] = total_time / self.performance_metrics["total_actions"]

    def get_performance_report(self) -> Dict[str, Any]:
        """
        Get a performance report
        """
        success_rate = (self.performance_metrics["successful_actions"] /
                       max(1, self.performance_metrics["total_actions"])) * 100

        return {
            "performance_metrics": self.performance_metrics,
            "success_rate": success_rate,
            "report_time": datetime.now().isoformat()
        }

    def log_safety_violation(self, violation_type: str, details: str):
        """
        Log a safety violation
        """
        self.performance_metrics["safety_violations"] += 1

        # Also log in action log for tracking
        violation_log = {
            "event_type": "safety_violation",
            "violation_type": violation_type,
            "details": details,
            "timestamp": datetime.now().isoformat()
        }

        self.action_log.append(violation_log)

    def get_action_history(self, action_type: str = None, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get action history, optionally filtered by type
        """
        if action_type:
            filtered_actions = [log for log in self.action_log if log.get("action_type") == action_type]
        else:
            filtered_actions = self.action_log

        return filtered_actions[-limit:]