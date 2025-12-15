"""
OpenAI Agents SDK chat agent for the Physical AI & Humanoid Robotics platform.
Implements a specialized agent for educational Q&A with curriculum knowledge.
"""

from typing import Dict, Any, List, Optional
from src.core.config import settings
from openai import OpenAI
from dataclasses import dataclass

@dataclass
class ToolResult:
    success: bool
    result: Any
    error: Optional[str] = None


class ChatAgent:
    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        
        # Create the agent with specific system instructions
        self.agent = self.client.beta.assistants.create(
            name="Physical AI Teaching Assistant",
            instructions=self._get_system_prompt(),
            model="gpt-4-turbo",  # Using GPT-4 Turbo for best performance
            tools=self._get_tools()
        )
        
    def _get_system_prompt(self) -> str:
        """
        Get the system prompt for the teaching assistant agent.
        """
        return """
        You are an expert teaching assistant for the Physical AI & Humanoid Robotics educational platform. 
        Your role is to answer student questions about the curriculum content, which includes:
        - Robot Operating System 2 (ROS 2)
        - Simulation environments (Gazebo, Unity ML-Agents)
        - NVIDIA Isaac Sim
        - Vision-Language-Action (VLA) models
        - Hardware platforms (Jetson Orin, Unitree Go1, etc.)

        Your responses should be:
        1. Accurate and based on the curriculum content
        2. Clear and educational
        3. Helpful but not doing assignments for the student
        4. Professional and encouraging

        Always cite sources when possible and acknowledge when you don't have sufficient information.
        If asked about implementation details beyond the curriculum, guide the student to appropriate resources.
        """
    
    def _get_tools(self) -> List[Dict[str, Any]]:
        """
        Define tools that the agent can use.
        For now, we'll include custom tools for navigation and information retrieval.
        """
        return [
            {
                "type": "function",
                "function": {
                    "name": "navigate_to_section",
                    "description": "Navigate to a specific section in the curriculum",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "chapter_id": {
                                "type": "string",
                                "description": "The ID of the chapter to navigate to"
                            },
                            "section_title": {
                                "type": "string",
                                "description": "The title of the section to navigate to"
                            }
                        },
                        "required": ["chapter_id", "section_title"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "search_curriculum",
                    "description": "Search the curriculum for specific information",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "The search query"
                            },
                            "max_results": {
                                "type": "integer",
                                "description": "Maximum number of results to return"
                            }
                        },
                        "required": ["query"]
                    }
                }
            }
        ]
    
    def navigate_to_section(self, chapter_id: str, section_title: str) -> ToolResult:
        """
        Tool to navigate to a specific section in the curriculum.
        """
        try:
            # In a real implementation, this would return a URL or reference to the section
            # For now, we'll just return a placeholder
            result = {
                "chapter_id": chapter_id,
                "section_title": section_title,
                "url": f"/docs/{chapter_id}#{section_title.replace(' ', '-').lower()}"
            }
            return ToolResult(success=True, result=result)
        except Exception as e:
            return ToolResult(success=False, result=None, error=str(e))
    
    def search_curriculum(self, query: str, max_results: int = 3) -> ToolResult:
        """
        Tool to search the curriculum for specific information.
        This would normally connect to our RAG service, but for now we'll simulate.
        """
        try:
            # In a real implementation, this would call our RAG service
            # For now, we'll return a placeholder result
            result = {
                "query": query,
                "results": [
                    {"title": "Relevant Topic", "chapter": "module-1/week-1-introduction", "relevance": 0.95},
                    {"title": "Another Topic", "chapter": "module-1/week-2-nodes-topics", "relevance": 0.87}
                ][:max_results]
            }
            return ToolResult(success=True, result=result)
        except Exception as e:
            return ToolResult(success=False, result=None, error=str(e))
    
    def chat(self, query: str, thread_id: Optional[str] = None) -> str:
        """
        Process a chat query using the agent.
        """
        try:
            # Create or use existing thread
            if not thread_id:
                thread = self.client.beta.threads.create()
            else:
                thread = self.client.beta.threads.retrieve(thread_id)
            
            # Add user message to thread
            self.client.beta.threads.messages.create(
                thread_id=thread.id,
                role="user",
                content=query
            )
            
            # Run the agent
            run = self.client.beta.threads.runs.create(
                thread_id=thread.id,
                assistant_id=self.agent.id
            )
            
            # Wait for the run to complete
            import time
            while run.status in ["queued", "in_progress"]:
                time.sleep(0.5)
                run = self.client.beta.threads.runs.retrieve(thread_id=thread.id, run_id=run.id)
            
            # Get the messages
            messages = self.client.beta.threads.messages.list(thread_id=thread.id)
            
            # Return the latest assistant message
            for message in messages.data:
                if message.role == "assistant":
                    return message.content[0].text.value  # Assuming text content
            
            return "I couldn't generate a response to your query."
            
        except Exception as e:
            print(f"Error in agent chat: {str(e)}")
            return "I encountered an error processing your request. Please try again."
    
    def cleanup(self):
        """
        Clean up the agent resources.
        """
        try:
            self.client.beta.assistants.delete(self.agent.id)
        except Exception as e:
            print(f"Error cleaning up agent: {str(e)}")