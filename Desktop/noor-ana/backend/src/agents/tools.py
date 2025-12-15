"""
Custom tools for the OpenAI Agents SDK.
These tools provide specific functionality for the Physical AI & Humanoid Robotics platform.
"""

from typing import Dict, Any, Optional
from src.core.config import settings
from openai import OpenAI
import json

class BookNavigationTool:
    """
    Tool for navigating to specific sections in the educational content.
    """
    
    def __init__(self):
        self.name = "navigate_to_section"
        self.description = "Navigate to a specific section in the curriculum"
    
    def get_tool_spec(self) -> Dict[str, Any]:
        """
        Get the specification for this tool in OpenAI's format.
        """
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
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
        }
    
    def execute(self, chapter_id: str, section_title: str) -> Dict[str, Any]:
        """
        Execute the navigation tool.
        """
        try:
            # In a real implementation, this would validate the chapter exists
            # and return the appropriate URL or reference
            result = {
                "success": True,
                "chapter_id": chapter_id,
                "section_title": section_title,
                "url": f"/docs/{chapter_id}#{section_title.replace(' ', '-').lower()}"
            }
            return result
        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }


class ContentSearchTool:
    """
    Tool for searching the curriculum content.
    """
    
    def __init__(self):
        self.name = "search_curriculum"
        self.description = "Search the curriculum for specific information"
    
    def get_tool_spec(self) -> Dict[str, Any]:
        """
        Get the specification for this tool in OpenAI's format.
        """
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query"
                        },
                        "max_results": {
                            "type": "integer",
                            "description": "Maximum number of results to return",
                            "default": 3
                        }
                    },
                    "required": ["query"]
                }
            }
        }
    
    def execute(self, query: str, max_results: int = 3) -> Dict[str, Any]:
        """
        Execute the search tool.
        In a real implementation, this would connect to our RAG service.
        """
        try:
            # In a real implementation, this would call our RAG service
            # For now, we'll return a placeholder result
            # This is where we would use the RAG service from rag.py
            
            result = {
                "success": True,
                "query": query,
                "max_results": max_results,
                "results": [
                    {
                        "title": "ROS 2 Fundamentals",
                        "chapter": "module-1/week-1-introduction",
                        "relevance": 0.95,
                        "preview": "ROS 2 is the second generation of the Robot Operating System..."
                    },
                    {
                        "title": "Nodes and Topics in ROS 2",
                        "chapter": "module-1/week-2-nodes-topics", 
                        "relevance": 0.87,
                        "preview": "Nodes are the fundamental building blocks of ROS 2 applications..."
                    },
                    {
                        "title": "Services and Actions",
                        "chapter": "module-1/week-3-services-actions",
                        "relevance": 0.78,
                        "preview": "While topics provide asynchronous communication, services provide synchronous request/reply..."
                    }
                ][:max_results]
            }
            return result
        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }


class CodeExampleTool:
    """
    Tool for retrieving code examples relevant to the query.
    """
    
    def __init__(self):
        self.name = "get_code_example"
        self.description = "Retrieve relevant code examples from the curriculum"
    
    def get_tool_spec(self) -> Dict[str, Any]:
        """
        Get the specification for this tool in OpenAI's format.
        """
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": {
                        "topic": {
                            "type": "string",
                            "description": "The topic for which to retrieve code examples"
                        },
                        "language": {
                            "type": "string",
                            "description": "Programming language (e.g., Python, C++)",
                            "default": "Python"
                        }
                    },
                    "required": ["topic"]
                }
            }
        }
    
    def execute(self, topic: str, language: str = "Python") -> Dict[str, Any]:
        """
        Execute the code example tool.
        """
        try:
            # In a real implementation, this would retrieve actual code examples
            # from the curriculum based on the topic and language
            examples = {
                "python": {
                    "publisher": '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
''',
                    "subscriber": '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
'''
                }
            }
            
            code = examples.get(language.lower(), {}).get(topic.lower())
            if not code:
                code = f"# Example code for {topic} in {language} would go here\npass"
            
            result = {
                "success": True,
                "topic": topic,
                "language": language,
                "code_example": code
            }
            return result
        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }


# Collection of all tools
def get_all_tools():
    """
    Get all available tools for the agent.
    """
    return [
        BookNavigationTool(),
        ContentSearchTool(), 
        CodeExampleTool()
    ]


def execute_tool(tool_name: str, **kwargs) -> Dict[str, Any]:
    """
    Execute a specific tool by name.
    """
    tools = {tool.name: tool for tool in get_all_tools()}
    
    if tool_name not in tools:
        return {
            "success": False,
            "error": f"Tool '{tool_name}' not found"
        }
    
    tool = tools[tool_name]
    try:
        # Extract parameters needed for the tool
        if tool_name == "navigate_to_section":
            return tool.execute(
                chapter_id=kwargs.get("chapter_id"),
                section_title=kwargs.get("section_title")
            )
        elif tool_name == "search_curriculum":
            return tool.execute(
                query=kwargs.get("query"),
                max_results=kwargs.get("max_results", 3)
            )
        elif tool_name == "get_code_example":
            return tool.execute(
                topic=kwargs.get("topic"),
                language=kwargs.get("language", "Python")
            )
        else:
            return {
                "success": False,
                "error": f"Tool '{tool_name}' execution not implemented"
            }
    except Exception as e:
        return {
            "success": False,
            "error": str(e)
        }