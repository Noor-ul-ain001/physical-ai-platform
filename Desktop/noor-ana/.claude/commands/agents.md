# /agents

The `/agents` command provides access to specialized AI agents for the Physical AI & Humanoid Robotics educational platform. Each agent is designed for specific tasks to assist in development, content creation, and platform management.

## Available Agents

### DocusaurusContentAgent
- **Purpose**: Generates MDX chapters for the curriculum
- **Command**: `/agents docusaurus-spec`
- **Examples**:
  - `/agents docusaurus-spec "Create Week 5: Gazebo Basics with code examples"`
  - `/agents docusaurus-spec "Generate content about ROS 2 services and actions"`

### RAGIntegrationAgent
- **Purpose**: Implements RAG features and optimizes search
- **Command**: `/agents rag-integration`
- **Examples**:
  - `/agents rag-integration "Optimize semantic search"`
  - `/agents rag-integration "Implement hallucination prevention"`

### PersonalizationAgent
- **Purpose**: Creates content personalization rules
- **Command**: `/agents personalization-spec`
- **Examples**:
  - `/agents personalization-spec "Generate rules for beginner users with Jetson access"`
  - `/agents personalization-spec "Adapt content for simulation-focused learners"`

### UIComponentAgent
- **Purpose**: Generates React components
- **Command**: `/agents ui-component`
- **Examples**:
  - `/agents ui-component "Create HardwareTable component"`
  - `/agents ui-component "Generate interactive code block"`

### DevOpsAgent
- **Purpose**: Manages deployment configurations
- **Command**: `/agents devops`
- **Examples**:
  - `/agents devops "Create Vercel deployment configuration"`
  - `/agents devops "Set up CI/CD pipeline for backend"`

## Usage

To invoke an agent, use the format:
```
/agents [agent-name] "[your request]"
```

The agent will process your request according to its specialized capabilities and return the appropriate output.

## Agent Registry

All agents are registered in `.specify/agents/registry.json` and their capabilities are defined in their respective directories under `.specify/agents/`.