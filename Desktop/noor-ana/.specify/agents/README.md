# Agent System Documentation

This directory contains specialized AI agents (subagents) for the Physical AI & Humanoid Robotics project, as defined in **Constitution Article IV: Reusable Intelligence & Agent Skills**.

## Overview

Agents are specialized AI assistants that handle distinct project domains with expertise. Each agent has:

- **Versioned prompts** - System prompts defining behavior and capabilities
- **Skill definitions** - Input/output contracts with test cases
- **Configuration** - Metadata and quality standards
- **Usage history** - Tracking for continuous improvement

## Quick Start

### List Available Agents

```bash
/agents
```

### Invoke an Agent

```bash
/agents <agent-id> "<task-description>"
```

Example:
```bash
/agents docusaurus-spec "Create MDX page for Chapter 2: Sensor Fusion with code examples and interactive diagrams"
```

### Create a New Agent

```bash
/agents create <agent-name>
```

## Available Agents

| Agent ID | Status | Description |
|----------|--------|-------------|
| **docusaurus-spec** | ✅ Active | Generates Docusaurus-compliant MDX from educational specifications |
| **rag-integration** | ✅ Active | Implements and tests RAG retrieval pipelines |
| **hardware-validation** | 📝 Draft | Validates hardware setup instructions and cost calculations |
| **ros2-code** | 📝 Draft | Generates and validates ROS 2 code snippets |

## Directory Structure

```
.specify/agents/
├── README.md                          # This file
├── registry.json                      # Master registry of all agents
├── docusaurus-spec/
│   ├── prompt.md                      # System prompt
│   ├── context.json                   # Configuration
│   ├── skills.md                      # Capability definitions
│   └── tests.md                       # Test scenarios
├── rag-integration/
│   ├── prompt.md
│   ├── context.json
│   ├── skills.md
│   └── tests.md
├── hardware-validation/
│   ├── prompt.md
│   ├── context.json
│   ├── skills.md
│   └── tests.md
└── ros2-code/
    ├── prompt.md
    ├── context.json
    ├── skills.md
    └── tests.md
```

## Agent Lifecycle

### 1. Creation

```bash
/agents create my-agent-name
```

Creates directory structure with template files.

### 2. Development (Draft Status)

- Write agent prompt defining role and capabilities
- Document skills with input/output contracts
- Create test scenarios
- Validate with example invocations

### 3. Activation

Update `status` in `registry.json` from `"draft"` to `"active"` after:
- Prompt is complete and tested
- Skills are documented with contracts
- Test scenarios pass
- At least 3 successful real-world uses

### 4. Evolution

- Version prompts when making significant changes (semantic versioning)
- Extract successful patterns from PHRs
- Refine skills based on usage
- Archive deprecated agents (don't delete - knowledge preservation)

## Constitutional Requirements

Per **Article IV**, all agents MUST:

1. **Single Responsibility** - Each agent has one clear domain
2. **Versioned Prompts** - Store prompts in `.specify/agents/<id>/prompt.md`
3. **Test Scenarios** - Document success/failure modes in `tests.md`
4. **Input/Output Contracts** - Define in `skills.md` and `registry.json`
5. **Knowledge Preservation** - Generate PHRs for every invocation
6. **Composability** - Skills can feed into other agents' inputs

## Agent Quality Standards

### Prompts

- Clear role definition
- Specific capabilities (skills)
- Explicit quality standards
- Constitutional alignment
- Example invocations

### Skills

- Named capability (verb-noun format: "generate-mdx", "validate-code")
- Input contract: What data/format is required
- Output contract: What is produced and in what format
- Test cases: 2-3 scenarios demonstrating success/failure
- Composability: How this skill integrates with others

### Tests

- Happy path: Expected successful scenario
- Edge cases: Boundary conditions, missing data
- Failure modes: What breaks and how to detect
- Quality validation: How to verify output meets standards

## Example: Docusaurus Spec Agent

### Input
```json
{
  "type": "chapter-generation",
  "chapter": "Chapter 3: Robot Kinematics",
  "learningObjectives": [
    "Understand forward kinematics",
    "Calculate inverse kinematics for 3-DOF arm"
  ],
  "codeExamples": ["DH parameters", "IK solver"],
  "interactiveElements": ["kinematic visualizer"]
}
```

### Output
```mdx
---
title: Chapter 3: Robot Kinematics
sidebar_position: 3
---

import KinematicVisualizer from '@site/src/components/KinematicVisualizer';

# Robot Kinematics

## Learning Objectives
- Understand forward kinematics and DH parameters
- Calculate inverse kinematics for 3-DOF robotic arm

[... content ...]

<KinematicVisualizer armType="3dof" />
```

### Skills Used
- `mdx-generation` - Create valid MDX structure
- `docusaurus-formatting` - Apply Docusaurus conventions
- `educational-content` - Structure for learning
- `react-component-integration` - Embed interactive components

## Best Practices

### Invoking Agents

✅ **Good**: Specific, clear task with context
```bash
/agents rag-integration "Implement RAG pipeline for Chapter 1-5 content with Qdrant Cloud, embedding model: text-embedding-3-small, chunk size: 512 tokens"
```

❌ **Bad**: Vague request
```bash
/agents rag-integration "set up chatbot"
```

### Creating Agents

✅ **Good**: Addresses specific, recurring need
- "Citation-validator: Ensures all RAG responses cite source sections"
- "Simulation-tester: Validates Gazebo/Isaac Sim code before publication"

❌ **Bad**: Too broad or one-off task
- "Helper-agent: Does various things"
- "Fix-this-bug: Specific to one incident"

### Agent Composition

Agents can work together. Example workflow:

1. **docusaurus-spec** generates initial MDX
2. **ros2-code** validates embedded ROS 2 snippets
3. **hardware-validation** verifies hardware instructions
4. **rag-integration** indexes content for chatbot

## Monitoring & Improvement

### Usage Tracking

`registry.json` tracks:
- `usageCount` - Total invocations
- `lastUsed` - Most recent use
- `version` - Current prompt version

### PHR Analysis

Monthly review:
1. Read PHRs from `history/prompts/<feature>/`
2. Identify successful patterns
3. Extract common failures
4. Update agent prompts/skills accordingly

### Metrics

Track (manually or via tooling):
- Success rate (task completed vs failed)
- Quality score (user satisfaction, output correctness)
- Reuse rate (same agent for multiple features)
- Composition rate (agents used together)

## Troubleshooting

### Agent Not Found

```
Error: Agent 'xyz' not found
```

**Solution**: Run `/agents` to list available agents. Use exact ID from registry.

### Agent Execution Failed

```
Error: Agent invocation failed - [details]
```

**Solution**:
1. Check agent prompt at `.specify/agents/<id>/prompt.md`
2. Verify input matches `inputContract` in registry
3. Review recent PHRs for similar failures
4. Update agent prompt if systematic issue

### Quality Issues

If agent produces low-quality output:
1. Review `tests.md` - are test scenarios representative?
2. Check PHRs - what patterns lead to poor output?
3. Refine prompt with specific quality standards
4. Add validation steps to prompt execution protocol
5. Consider splitting agent if doing too much

## Future Extensions

Planned improvements:
- Automated testing framework for agents
- Agent performance dashboards
- Skill marketplace (share agents across projects)
- Agent composition language (chain agents declaratively)

## References

- **Constitution Article IV**: Reusable Intelligence & Agent Skills (.specify/memory/constitution.md:146-180)
- **PHR Template**: .specify/templates/phr-template.prompt.md
- **Agent Command**: .claude/commands/agents.md
