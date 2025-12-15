"""
Personalization service for the Physical AI & Humanoid Robotics platform.
This service applies content adaptation rules based on user profile and preferences.
"""

from typing import Dict, List, Any, Optional
from enum import Enum
from dataclasses import dataclass
import re

class HardwareLevel(Enum):
    NONE = "none"
    CLOUD = "cloud"
    JETSON = "jetson"
    FULL_ROBOT = "full_robot"

class ExperienceLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

@dataclass
class PersonalizationRules:
    show_sections: List[str]
    hide_sections: List[str]
    code_complexity: str  # "simple", "standard", "advanced"
    hardware_instructions: str  # "cloud", "jetson", "full_robot"

@dataclass
class UserProfile:
    hardware_access: HardwareLevel
    experience_level: ExperienceLevel
    learning_goals: List[str]

class PersonalizationService:
    def __init__(self):
        # Define personalization rules based on user profile
        # These rules determine which content sections to show/hide based on:
        # - Hardware access level
        # - Experience level
        # - Learning goals
        self.rules_map = {
            (HardwareLevel.NONE, ExperienceLevel.BEGINNER): {
                "show_sections": ["cloud_simulation", "basic_concepts", "visual_explanations"],
                "hide_sections": ["jetson_setup", "robot_integration", "advanced_troubleshooting"],
                "code_complexity": "simple",
                "hardware_instructions": "cloud"
            },
            (HardwareLevel.NONE, ExperienceLevel.INTERMEDIATE): {
                "show_sections": ["cloud_simulation", "intermediate_concepts", "visual_explanations"],
                "hide_sections": ["jetson_hardware", "robot_integration"],
                "code_complexity": "standard",
                "hardware_instructions": "cloud"
            },
            (HardwareLevel.NONE, ExperienceLevel.ADVANCED): {
                "show_sections": ["cloud_simulation", "advanced_concepts", "algorithm_details"],
                "hide_sections": ["basic_explanations", "safety_warnings"],
                "code_complexity": "advanced",
                "hardware_instructions": "cloud"
            },
            (HardwareLevel.JETSON, ExperienceLevel.BEGINNER): {
                "show_sections": ["jetson_setup", "basic_concepts", "visual_explanations", "simulation"],
                "hide_sections": ["robot_integration", "advanced_troubleshooting", "custom_hardware"],
                "code_complexity": "simple",
                "hardware_instructions": "jetson"
            },
            (HardwareLevel.JETSON, ExperienceLevel.INTERMEDIATE): {
                "show_sections": ["jetson_setup", "intermediate_concepts", "simulation", "cloud_options"],
                "hide_sections": ["basic_explanations", "advanced_troubleshooting"],
                "code_complexity": "standard",
                "hardware_instructions": "jetson"
            },
            (HardwareLevel.JETSON, ExperienceLevel.ADVANCED): {
                "show_sections": ["jetson_optimization", "advanced_concepts", "custom_hardware", "simulation"],
                "hide_sections": ["basic_explanations", "safety_warnings"],
                "code_complexity": "advanced",
                "hardware_instructions": "jetson"
            },
            (HardwareLevel.FULL_ROBOT, ExperienceLevel.BEGINNER): {
                "show_sections": ["robot_basics", "safety", "basic_control", "simulation"],
                "hide_sections": ["advanced_troubleshooting", "custom_hardware"],
                "code_complexity": "simple",
                "hardware_instructions": "full_robot"
            },
            (HardwareLevel.FULL_ROBOT, ExperienceLevel.INTERMEDIATE): {
                "show_sections": ["robot_control", "intermediate_concepts", "simulation", "jetson_options"],
                "hide_sections": ["basic_safety", "advanced_troubleshooting"],
                "code_complexity": "standard",
                "hardware_instructions": "full_robot"
            },
            (HardwareLevel.FULL_ROBOT, ExperienceLevel.ADVANCED): {
                "show_sections": ["robot_control", "advanced_concepts", "custom_hardware", "simulation"],
                "hide_sections": ["basic_safety", "beginner_explanations"],
                "code_complexity": "advanced",
                "hardware_instructions": "full_robot"
            }
        }

        # Define rules for specific learning goals
        self.goal_rules = {
            "ROS 2 Development": {
                "show_sections": ["ros_architecture", "nodes_topics", "services_actions"],
                "hide_sections": ["simulation_details", "hardware_specifics"]
            },
            "Simulation Environments": {
                "show_sections": ["gazebo_tutorials", "unity_ml_agents", "simulation_optimization"],
                "hide_sections": ["hardware_setup", "robot_specifics"]
            },
            "NVIDIA Isaac Sim": {
                "show_sections": ["isaac_setup", "synthetic_data", "robot_learning"],
                "hide_sections": ["other_simulators", "basic_simulation"]
            },
            "VLA Models": {
                "show_sections": ["vision_language", "action_models", "embodied_ai"],
                "hide_sections": ["basic_programming", "simulation_basics"]
            }
        }

    def get_personalization_rules(self, user_profile: UserProfile) -> PersonalizationRules:
        """
        Get personalization rules for the given user profile.
        Combines base rules based on hardware and experience with rules based on learning goals.
        """
        # Get base rules based on hardware access and experience level
        key = (user_profile.hardware_access, user_profile.experience_level)
        base_rules = self.rules_map.get(key, self._get_default_rules())

        # Start with base rules
        show_sections = set(base_rules["show_sections"])
        hide_sections = set(base_rules["hide_sections"])
        code_complexity = base_rules["code_complexity"]
        hardware_instructions = base_rules["hardware_instructions"]

        # Apply learning goal-specific rules
        for goal in user_profile.learning_goals:
            goal_specific_rules = self.goal_rules.get(goal, {})
            if "show_sections" in goal_specific_rules:
                show_sections.update(goal_specific_rules["show_sections"])
            if "hide_sections" in goal_specific_rules:
                hide_sections.update(goal_specific_rules["hide_sections"])

        # Ensure sections marked to hide are not also marked to show
        show_sections = show_sections - hide_sections

        return PersonalizationRules(
            show_sections=list(show_sections),
            hide_sections=list(hide_sections),
            code_complexity=code_complexity,
            hardware_instructions=hardware_instructions
        )

    def _get_default_rules(self) -> Dict[str, Any]:
        """
        Get default personalization rules when no specific rules match.
        """
        return {
            "show_sections": ["basic_concepts", "visual_explanations"],
            "hide_sections": ["jetson_setup", "robot_integration", "advanced_troubleshooting"],
            "code_complexity": "standard",
            "hardware_instructions": "cloud"
        }

    def apply_rules_to_content(self, content: str, rules: PersonalizationRules) -> str:
        """
        Apply personalization rules to content by showing/hiding sections.
        """
        # This is a simplified implementation - a full version would use more sophisticated
        # content parsing and modification techniques
        result = content
        
        # Remove hidden sections (marked with specific comments)
        for section in rules.hide_sections:
            # Pattern to match <!-- hide-section:{section} --> ... <!-- end-hide -->
            pattern = f"<!--\\s*hide-section:{re.escape(section)}\\s*-->(.*?)<!--\\s*end-hide\\s*-->"
            result = re.sub(pattern, "", result, flags=re.DOTALL)
        
        # Modify code complexity (simplified)
        if rules.code_complexity == "simple":
            # Replace advanced code examples with simpler ones
            result = self._simplify_code_examples(result)
        elif rules.code_complexity == "advanced":
            # Replace basic examples with advanced ones
            result = self._advanced_code_examples(result)
        
        # Adjust hardware instructions
        result = self._adjust_hardware_instructions(result, rules.hardware_instructions)
        
        return result

    def _simplify_code_examples(self, content: str) -> str:
        """
        Simplify code examples in the content.
        """
        # This is a placeholder implementation
        # In a real system, this would replace complex code with simpler alternatives
        content = content.replace("# Advanced implementation", "# Simple implementation")
        content = content.replace("complex_function()", "simple_function()")
        return content

    def _advanced_code_examples(self, content: str) -> str:
        """
        Enhance code examples with more advanced concepts.
        """
        # This is a placeholder implementation
        # In a real system, this would replace basic code with advanced alternatives
        content = content.replace("# Basic implementation", "# Advanced implementation")
        content = content.replace("basic_function()", "advanced_function()")
        return content

    def _adjust_hardware_instructions(self, content: str, hardware_level: str) -> str:
        """
        Adjust hardware-specific instructions based on user's hardware access.
        """
        # Replace hardware-specific instructions
        if hardware_level == "cloud":
            content = content.replace("{hardware_instructions}", "Follow the cloud simulation instructions")
        elif hardware_level == "jetson":
            content = content.replace("{hardware_instructions}", "Follow the Jetson Orin setup instructions")
        elif hardware_level == "full_robot":
            content = content.replace("{hardware_instructions}", "Follow the full robot platform instructions")
        
        return content