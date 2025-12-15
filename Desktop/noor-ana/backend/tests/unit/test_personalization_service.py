import pytest
from src.services.personalization import PersonalizationService, UserProfile, HardwareLevel, ExperienceLevel


class TestPersonalizationService:
    """Unit tests for the Personalization Service"""
    
    def test_get_personalization_rules_beginner_no_hardware(self):
        """Test personalization rules for beginner with no hardware"""
        service = PersonalizationService()
        
        user_profile = UserProfile(
            hardware_access=HardwareLevel.NONE,
            experience_level=ExperienceLevel.BEGINNER,
            learning_goals=[]
        )
        
        rules = service.get_personalization_rules(user_profile)
        
        assert "cloud_simulation" in rules.show_sections
        assert "basic_concepts" in rules.show_sections
        assert "jetson_setup" in rules.hide_sections
        assert rules.code_complexity == "simple"
        assert rules.hardware_instructions == "cloud"
    
    def test_get_personalization_rules_advanced_with_robot(self):
        """Test personalization rules for advanced user with full robot"""
        service = PersonalizationService()
        
        user_profile = UserProfile(
            hardware_access=HardwareLevel.FULL_ROBOT,
            experience_level=ExperienceLevel.ADVANCED,
            learning_goals=["ROS 2 Development"]
        )
        
        rules = service.get_personalization_rules(user_profile)
        
        assert "robot_control" in rules.show_sections
        assert "advanced_concepts" in rules.show_sections
        assert "basic_safety" in rules.hide_sections
        assert rules.code_complexity == "advanced"
        assert rules.hardware_instructions == "full_robot"
        
        # Verify learning goal-specific rules were applied
        assert "ros_architecture" in rules.show_sections