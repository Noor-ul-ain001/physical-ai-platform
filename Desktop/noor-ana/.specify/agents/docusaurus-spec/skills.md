{
  "agent_id": "docusaurus-content-agent",
  "skills": [
    {
      "id": "mdx-generation",
      "name": "MDX Generation",
      "description": "Generate content in MDX format with proper structure and components",
      "parameters": {
        "topic": {
          "type": "string",
          "description": "The topic to create content about",
          "required": true
        },
        "difficulty": {
          "type": "string",
          "enum": ["beginner", "intermediate", "advanced"],
          "description": "The difficulty level of the content",
          "default": "beginner"
        },
        "learning_objectives": {
          "type": "array",
          "items": {
            "type": "string"
          },
          "description": "List of learning objectives for the chapter"
        }
      }
    },
    {
      "id": "docusaurus-formatting",
      "name": "Docusaurus Formatting",
      "description": "Apply proper Docusaurus-specific formatting with frontmatter and navigation",
      "parameters": {
        "title": {
          "type": "string",
          "description": "The title of the document",
          "required": true
        },
        "sidebar_position": {
          "type": "number",
          "description": "The position in the sidebar navigation",
          "required": false
        },
        "module": {
          "type": "string",
          "description": "The module this content belongs to",
          "required": true
        }
      }
    },
    {
      "id": "curriculum-standards",
      "name": "Curriculum Standards",
      "description": "Ensure content aligns with curriculum standards and learning objectives",
      "parameters": {
        "module": {
          "type": "string",
          "description": "The module this content belongs to",
          "required": true
        },
        "standards": {
          "type": "array",
          "items": {
            "type": "string"
          },
          "description": "List of standards that the content should meet"
        }
      }
    },
    {
      "id": "code-examples",
      "name": "Code Examples",
      "description": "Generate relevant code examples in Python, C++, or other appropriate languages",
      "parameters": {
        "language": {
          "type": "string",
          "enum": ["python", "cpp", "other"],
          "description": "The programming language for examples",
          "default": "python"
        },
        "topic": {
          "type": "string",
          "description": "The topic to demonstrate with code",
          "required": true
        },
        "complexity": {
          "type": "string",
          "enum": ["simple", "intermediate", "advanced"],
          "description": "The complexity level of the example",
          "default": "simple"
        }
      }
    },
    {
      "id": "interactive-components",
      "name": "Interactive Components",
      "description": "Integrate interactive components to enhance learning",
      "parameters": {
        "components": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["HardwareTable", "InteractiveCodeBlock", "DiagramViewer"]
          },
          "description": "List of interactive components to include"
        }
      }
    }
  ]
}