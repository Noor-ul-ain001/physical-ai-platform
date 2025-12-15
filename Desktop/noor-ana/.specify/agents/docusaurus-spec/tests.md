# Docusaurus Spec Agent - Test Scenarios

This document contains test scenarios for validating the Docusaurus Spec Agent's capabilities.

## Test Scenario 1: Basic Chapter Generation

### Input
```json
{
  "chapter": "Chapter 2: Python for Robotics",
  "learningObjectives": [
    "Use NumPy for matrix operations",
    "Implement basic control loops",
    "Handle sensor data streams"
  ],
  "keyConcepts": ["NumPy arrays", "Control loops", "Data processing"],
  "codeExamples": [
    "Matrix multiplication with NumPy",
    "PID controller implementation"
  ],
  "interactiveElements": ["Code playground"],
  "prerequisites": ["Python basics", "Linear algebra"],
  "estimatedTime": "45 minutes"
}
```

### Expected Output Structure

```mdx
---
title: Chapter 2 - Python for Robotics
sidebar_position: 2
description: Master Python tools essential for robotics programming
keywords: [python, numpy, robotics, control-loops]
estimatedTime: 45 minutes
---

import CodePlayground from '@site/src/components/CodePlayground';

# Python for Robotics

:::info Prerequisites
- Python basics (variables, functions, classes)
- Linear algebra (vectors, matrices)

Need a refresher? See [Python Appendix](#) or [Math Fundamentals](#).
:::

## Learning Objectives

By the end of this chapter, you will:
- ✅ Use NumPy for matrix operations in robotics applications
- ✅ Implement basic control loops for robot motion
- ✅ Handle sensor data streams efficiently

[... sections with code examples ...]

## Practice

<CodePlayground
  initialCode="# Implement a simple PID controller"
  language="python"
/>
```

### Validation Checklist

- [ ] Valid MDX syntax (no parse errors)
- [ ] Frontmatter includes all standard fields
- [ ] Learning objectives are measurable
- [ ] Code examples are complete and runnable
- [ ] Interactive element is included
- [ ] Prerequisites are clearly stated
- [ ] Heading hierarchy is proper (h1 → h2 → h3, no skips)

### Success Criteria

✅ **Pass**: All validation items checked, MDX renders in Docusaurus without errors

---

## Test Scenario 2: Advanced Chapter with Personalization

### Input
```json
{
  "chapter": "Chapter 7: Advanced Sensor Fusion",
  "learningObjectives": [
    "Implement Extended Kalman Filter",
    "Fuse IMU and vision data",
    "Handle sensor uncertainties"
  ],
  "keyConcepts": ["EKF", "Multi-modal fusion", "Uncertainty propagation"],
  "targetAudience": "advanced",
  "codeExamples": ["EKF implementation", "Sensor fusion pipeline"],
  "interactiveElements": ["EKF simulator", "Personalization button"],
  "mathHeavy": true
}
```

### Expected Output Structure

```mdx
---
title: Chapter 7 - Advanced Sensor Fusion
sidebar_position: 7
description: Master multi-sensor fusion with Extended Kalman Filters
difficulty: advanced
prerequisites: ['kalman-filters', 'linear-algebra', 'probability']
---

import EKFSimulator from '@site/src/components/EKFSimulator';
import PersonalizationButton from '@site/src/components/PersonalizationButton';

# Advanced Sensor Fusion

<PersonalizationButton chapter="advanced-sensor-fusion" />

:::warning Advanced Content
This chapter involves heavy mathematics (Jacobians, covariance matrices). Ensure you're comfortable with linear algebra and probability theory.
:::

[... content with math formulas, EKF derivation ...]

## Interactive EKF Simulation

<EKFSimulator
  sensors={['imu', 'camera']}
  visualization="3d"
/>
```

### Validation Checklist

- [ ] `difficulty: advanced` in frontmatter
- [ ] Personalization button included
- [ ] Advanced warning admonition
- [ ] Math content is rigorous but explained
- [ ] Multiple interactive elements
- [ ] Prerequisites clearly listed

### Success Criteria

✅ **Pass**: Advanced content appropriately flagged, personalization hooks present, interactivity functional

---

## Test Scenario 3: Error Handling - Missing Information

### Input
```json
{
  "chapter": "Chapter X: Unknown Topic",
  "learningObjectives": []
}
```

### Expected Behavior

Agent should:
1. Detect missing required fields (`learningObjectives` is empty)
2. Request clarification or infer minimal structure:

```mdx
---
title: Chapter X - Unknown Topic
sidebar_position: 0
description: [NEEDS CLARIFICATION: chapter_description]
---

# [NEEDS CLARIFICATION: chapter_title]

## Learning Objectives

[NEEDS CLARIFICATION: learning_objectives]

## Introduction

[NEEDS CLARIFICATION: chapter_content]
```

### Validation Checklist

- [ ] Placeholders clearly marked with `[NEEDS CLARIFICATION: field_name]`
- [ ] Valid MDX structure maintained
- [ ] Agent reports missing information in response

### Success Criteria

✅ **Pass**: Agent doesn't fail, produces valid MDX with clear placeholders, reports issues

---

## Test Scenario 4: Component Integration

### Input
```json
{
  "chapter": "Chapter 4: ROS 2 Visualization",
  "learningObjectives": ["Visualize robot state", "Debug with RViz2"],
  "interactiveElements": [
    {
      "type": "ROSGraphVisualizer",
      "props": {"nodes": ["talker", "listener"], "topics": ["chatter"]}
    },
    {
      "type": "RViz2Embed",
      "props": {"robotModel": "turtlebot3"}
    }
  ]
}
```

### Expected Output

```mdx
import ROSGraphVisualizer from '@site/src/components/ROSGraphVisualizer';
import RViz2Embed from '@site/src/components/RViz2Embed';

[... content ...]

## ROS Graph

<ROSGraphVisualizer
  nodes={['talker', 'listener']}
  topics={['chatter']}
/>

[... more content ...]

## RViz2 Simulation

<RViz2Embed robotModel="turtlebot3" />
```

### Validation Checklist

- [ ] Import statements at top of file
- [ ] Components used in correct locations
- [ ] Props correctly formatted (JSX syntax)
- [ ] Component names match available library

### Success Criteria

✅ **Pass**: All components imported, properly integrated, valid JSX syntax

---

## Test Scenario 5: Code Syntax Validation

### Input
```json
{
  "chapter": "Chapter 3: ROS 2 Services",
  "codeExamples": [
    {
      "language": "python",
      "description": "Service server implementation",
      "code": "import rclpy\nfrom example_interfaces.srv import AddTwoInts\n\nclass MinimalService(rclpy.node.Node):\n    def __init__(self):\n        super().__init__('minimal_service')\n        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)\n\n    def callback(self, request, response):\n        response.sum = request.a + request.b\n        return response"
    }
  ]
}
```

### Expected Output

```mdx
## Service Server Implementation

```python
import rclpy
from example_interfaces.srv import AddTwoInts

class MinimalService(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

:::tip Running the Service
Start the service server with:
```bash
ros2 run <package_name> service_server
```
:::
```

### Validation Checklist

- [ ] Code block has language tag (```python)
- [ ] Code is properly indented
- [ ] No syntax errors (valid Python)
- [ ] Helpful context provided (tip admonition)

### Success Criteria

✅ **Pass**: Code is syntactically valid, properly formatted, includes usage instructions

---

## Failure Mode Tests

### FM-1: Invalid Component Reference

**Input**: Request for non-existent component "SuperWidget"

**Expected**:
- Placeholder comment: `{/* TODO: Implement <SuperWidget /> */}`
- Warning in agent response: "Component 'SuperWidget' not found in library"

### FM-2: Broken Heading Hierarchy

**Input**: Content jumps from h1 to h4

**Expected**:
- Agent auto-corrects to proper hierarchy (h1 → h2 → h3)
- Or warns about skipped levels

### FM-3: Unclosed JSX Tag

**Input**: Component usage like `<Visualizer prop="value"`

**Expected**:
- Validation fails
- Error message: "Unclosed JSX tag detected: <Visualizer>"
- Suggestion: "Ensure all components are properly closed"

---

## Performance Benchmarks

- **Basic chapter (Scenario 1)**: < 30 seconds generation time
- **Advanced chapter (Scenario 2)**: < 60 seconds generation time
- **MDX validation**: < 1 second
- **Component integration (Scenario 4)**: < 45 seconds

---

## Continuous Testing

Run these scenarios:
- **Weekly**: All test scenarios to catch regressions
- **After prompt updates**: Full test suite
- **Before marking agent "active"**: 100% pass rate required

## Test Results Log

| Date | Scenario | Result | Notes |
|------|----------|--------|-------|
| 2025-12-15 | All | Not Run | Agent created, awaiting first test |

---

## Adding New Tests

When adding new test scenarios:
1. Include clear input (JSON or description)
2. Show expected output structure
3. Define validation checklist
4. State success criteria
5. Update test results log after running
