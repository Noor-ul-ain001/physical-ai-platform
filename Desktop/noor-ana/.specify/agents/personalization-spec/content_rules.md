# Personalization Content Rules

This document defines the content adaptation rules for the Physical AI & Humanoid Robotics educational platform based on user profiles and preferences.

## Rule Categories

### 1. Hardware-Based Adaptations

#### 1.1 No Hardware Access
- **Show Sections**: Cloud simulation, virtual environments, online robotics platforms
- **Hide Sections**: Physical hardware setup, hardware-specific troubleshooting, real-world deployment
- **Code Complexity**: Focus on simulation-based examples
- **Examples**: Use Gazebo and Isaac Sim examples rather than real hardware

#### 1.2 Jetson Orin Access
- **Show Sections**: Jetson setup, optimization, ROS 2 on embedded systems
- **Hide Sections**: Full robot kinematics (unless simulated), complex locomotion
- **Code Complexity**: Include performance optimization examples
- **Examples**: Jetson-specific code examples, power management

#### 1.3 Full Robot Access
- **Show Sections**: Real hardware integration, kinematics, locomotion, safety procedures
- **Hide Sections**: Simulation-only content
- **Code Complexity**: Advanced control algorithms, sensor fusion
- **Examples**: Full robot control, gait generation, sensor integration

### 2. Experience-Based Adaptations

#### 2.1 Beginner
- **Content Focus**: Fundamentals, basic concepts, visual explanations
- **Code Examples**: Simple functions, commented extensively, step-by-step
- **Mathematical Complexity**: Minimal, focus on concepts
- **Prerequisites**: Ensure foundational concepts are covered first
- **Assessment**: Basic quizzes, multiple choice

#### 2.2 Intermediate
- **Content Focus**: Implementation details, common patterns, best practices
- **Code Examples**: Complete examples, moderate complexity, good documentation
- **Mathematical Complexity**: Moderate, necessary formulas and concepts
- **Prerequisites**: Assumes basic understanding of concepts
- **Assessment**: Hands-on exercises, short answer questions

#### 2.3 Advanced
- **Content Focus**: Optimization, advanced algorithms, cutting-edge techniques
- **Code Examples**: Production-ready code, performance considerations
- **Mathematical Complexity**: Detailed equations, advanced concepts
- **Prerequisites**: Assumes comprehensive foundational knowledge
- **Assessment**: Complex projects, research-based assignments

### 3. Goal-Based Adaptations

#### 3.1 ROS 2 Development Focus
- **Emphasize**: ROS 2 architecture, nodes, topics, services, actions
- **De-emphasize**: Hardware-specific content, simulation details
- **Examples**: ROS 2 packages, launch files, parameter management

#### 3.2 Simulation Focus
- **Emphasize**: Gazebo, Isaac Sim, Unity ML-Agents, synthetic data generation
- **De-emphasize**: Real-world deployment, hardware limitations
- **Examples**: Physics simulation, sensor modeling, environment creation

#### 3.3 Isaac Sim Focus
- **Emphasize**: Isaac Sim workflows, USD format, synthetic data generation
- **De-emphasize**: Other simulators, basic simulation concepts
- **Examples**: USD scene creation, synthetic dataset generation

#### 3.4 VLA Models Focus
- **Emphasize**: Vision-language-action models, multimodal learning
- **De-emphasize**: Basic robotics controls, kinematics
- **Examples**: LLM integration, perception-action loops, embodied AI

### 4. Dynamic Adaptations Based on Progress

#### 4.1 Performance-Based Adjustments
- **Struggling Learner**: Provide additional explanations, simpler examples, more practice
- **Advanced Learner**: Offer enrichment material, advanced challenges, skip basics
- **Standard Learner**: Follow standard curriculum path

#### 4.2 Engagement-Based Adjustments
- **Low Engagement**: Provide more interactive elements, reduce content density
- **High Engagement**: Offer deeper dives, additional challenges

## Implementation Guidelines

### 1. Content Tagging
All curriculum content must be tagged with:
- `hardware-level`: "none", "jetson", "full-robot"
- `experience-level`: "beginner", "intermediate", "advanced" 
- `goal-area`: "ros2", "simulation", "isaac", "vla", etc.
- `prerequisites`: List of required concepts
- `learning-objectives`: Specific objectives addressed

### 2. Rule Application Order
1. Apply experience-based rules first
2. Apply hardware-based rules second
3. Apply goal-based rules third
4. Apply dynamic rules last

### 3. Fallback Logic
- If no personalization rules apply, show standard content
- If user profile is incomplete, default to beginner/no hardware
- Always preserve core learning objectives

## Quality Assurance

### 1. Testing Requirements
- All personalization pathways must achieve same learning outcomes
- No pathway should be significantly shorter/longer than others
- All pathways must be reviewed by domain experts

### 2. Performance Metrics
- Personalization rule application speed
- User engagement with personalized content
- Learning outcome achievement across pathways
- User satisfaction with personalization