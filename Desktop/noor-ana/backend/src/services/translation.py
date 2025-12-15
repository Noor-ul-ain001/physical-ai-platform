"""
Translation service for the Physical AI & Humanoid Robotics platform.
Implements Urdu translation with preservation of code blocks and technical terms.
"""

from typing import Dict, List, Tuple
from dataclasses import dataclass
import re
import html

@dataclass
class TranslationResult:
    translated_content: str
    preserved_code_blocks: List[str]
    preserved_links: List[str]

class TranslationService:
    def __init__(self, api_key: str = None):
        """
        Initialize the translation service.
        In a real implementation, this would connect to Google Translate API.
        For now, we'll use a mock implementation.
        """
        self.api_key = api_key

        # Technical glossary for domain-specific terms
        self.technical_glossary = {
            "ROS 2": "ROS 2",  # Keep acronym as is
            "Gazebo": "Gazebo",
            "Unity ML-Agents": "Unity ML-ایجنٹس",
            "Isaac Sim": "آئزک سیم",
            "VLA Models": "VLA ماڈلز",
            "Jetson Orin": "جیٹسن اورن",
            "Unitree Go1": "یونی ٹری گو1",
            "NVIDIA Isaac": "این وی ڈی اے آئزک",
            "Humanoid Robotics": "ہیومنوڈ روبوٹکس",
            "Embodied Intelligence": "ایم بیوڈ انٹیلی جنس",
            "Physical AI": "فزیکل اے آئی",
            "Robot Operating System": "روبوٹ آپریٹنگ سسٹم",
            "Simulation": "سمولیشن",
            "Deep Learning": "ڈیپ لرننگ",
            "Machine Learning": "مشین لرننگ",
            "Artificial Intelligence": "مصنوعی ذہانت",
            "Neural Network": "نیورل نیٹ ورک",
            "Algorithm": "الگورتھم",
            "Data Structure": "ڈیٹا سٹرکچر",
            "Programming": "پروگرامنگ",
            "Hardware": "ہارڈ ویئر",
            "Software": "سافٹ ویئر",
            "AI": "ذہانت",
            "Robot": "روبوٹ"
        }

    async def translate_content(self, content: str, target_language: str = "ur") -> TranslationResult:
        """
        Translate content to the target language while preserving code blocks and links.
        """
        if target_language != "ur":
            raise ValueError("Currently only Urdu translation is supported")
        
        # Extract and preserve code blocks
        code_blocks, content_with_placeholders = self._extract_code_blocks(content)
        
        # Extract and preserve links
        links, content_with_link_placeholders = self._extract_links(content_with_placeholders)
        
        # Preserve technical terms using glossary
        content_with_glossary = self._apply_glossary(content_with_link_placeholders)
        
        # In a real implementation, this would call the Google Translate API
        # translated_text = self._translate_text(content_with_glossary, target_language)
        # For now, we'll simulate the translation
        translated_text = self._simulate_urdu_translation(content_with_glossary)
        
        # Restore preserved code blocks
        final_content = self._restore_code_blocks(translated_text, code_blocks)
        final_content = self._restore_links(final_content, links)
        
        return TranslationResult(
            translated_content=final_content,
            preserved_code_blocks=code_blocks,
            preserved_links=links
        )
    
    def _extract_code_blocks(self, content: str) -> Tuple[List[str], str]:
        """
        Extract code blocks from content and replace with placeholders.
        """
        code_blocks = []
        placeholder_pattern = r'(```[\s\S]*?```|`[^`\n]+`)'
        
        def replace_code_block(match):
            code_block = match.group(0)
            code_blocks.append(code_block)
            block_index = len(code_blocks) - 1
            return f"__CODE_BLOCK_{block_index}__"
        
        content_with_placeholders = re.sub(placeholder_pattern, replace_code_block, content)
        return code_blocks, content_with_placeholders
    
    def _restore_code_blocks(self, content: str, code_blocks: List[str]) -> str:
        """
        Restore code blocks back into content using placeholders.
        """
        for i, code_block in enumerate(code_blocks):
            placeholder = f"__CODE_BLOCK_{i}__"
            content = content.replace(placeholder, code_block)
        return content

    def _extract_links(self, content: str) -> Tuple[List[str], str]:
        """
        Extract links from content and replace with placeholders.
        """
        links = []
        # Pattern to match markdown links [text](url) and plain URLs
        link_pattern = r'(\[([^\]]+)\]\(([^)]+)\)|https?://[^\s)]+)'
        
        def replace_link(match):
            full_match = match.group(0)
            links.append(full_match)
            link_index = len(links) - 1
            return f"__LINK_{link_index}__"
        
        content_with_placeholders = re.sub(link_pattern, replace_link, content)
        return links, content_with_placeholders
    
    def _restore_links(self, content: str, links: List[str]) -> str:
        """
        Restore links back into content using placeholders.
        """
        for i, link in enumerate(links):
            placeholder = f"__LINK_{i}__"
            content = content.replace(placeholder, link)
        return content

    def _apply_glossary(self, content: str) -> str:
        """
        Apply the technical glossary to preserve technical terms.
        """
        # Create a temporary placeholder system for glossary terms
        temp_placeholders = {}
        temp_content = content
        
        # Sort glossary by length (descending) to avoid partial replacements
        sorted_glossary = sorted(self.technical_glossary.items(), key=lambda x: len(x[0]), reverse=True)
        
        for eng_term, urdu_term in sorted_glossary:
            # Create a temporary placeholder for the English term
            placeholder = f"__GLOSSARY_{len(temp_placeholders)}__"
            temp_placeholders[placeholder] = f"{eng_term} ({urdu_term})"
            temp_content = temp_content.replace(eng_term, placeholder)
        
        # Now restore the glossary terms with their Urdu translations
        result = temp_content
        for placeholder, replacement in temp_placeholders.items():
            result = result.replace(placeholder, replacement)
        
        return result

    def _translate_text(self, text: str, target_language: str) -> str:
        """
        Translate text using Google Translate API.
        This is a placeholder for the actual API call.
        """
        # In a real implementation, this would call the Google Translate API
        # response = self.client.translate(text, target_language=target_language)
        # return response['translatedText']
        
        # For now, we'll just return a simulation
        return text

    def _simulate_urdu_translation(self, text: str) -> str:
        """
        Simulate Urdu translation for demonstration purposes.
        This is not a real translation but demonstrates the concept.
        """
        # This is just a simulation - a real implementation would call an actual translation API
        # For demo purposes, we'll just append a note
        return f"[URDU SIMULATION] {text} [TRANSLATION END]"
    
    async def translate_text_segment(self, text: str, target_language: str = "ur") -> str:
        """
        Translate a single text segment (used for translating chunks).
        """
        return await self.translate_content(text, target_language)