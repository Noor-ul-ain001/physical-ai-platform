---
id: 1
title: Amend Constitution to v3.1 and Sync Templates
stage: constitution
date: 2025-12-01
surface: agent
model: gemini
feature: none
branch: 001-rag-chatbot-book
user: user
command: /sp.constitution
labels: [constitution, template, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
  null
---

## Prompt

/sp.constitution # **Gemini-Powered Physical AI & Humanoid Robotics Textbook Constitution**
## **Version 3.1 - Gemini Edition with Color Scheme**

## 🎨 **Design System & Brand Identity**

### **Color Palette**
```yaml
Primary Colors:
  Primary: #B279A4 (Muted Purple)
  Secondary: #4A4A4A (Charcoal Gray)
  Accent: #79B2A4 (Muted Teal - Complementary)
  Neutral: #F8F9FA (Light Gray)
  Text: #2D3748 (Dark Gray)
  Background: #FFFFFF (White)

Color Applications:
  Primary Brand: #B279A4
  Headers & Titles: #B279A4
  Buttons & CTAs: #B279A4 (solid)
  Secondary Elements: #79B2A4
  Borders & Dividers: #E2E8F0
  Text Body: #4A4A4A
  Code Blocks: #F7FAFC
```

### **Design Principles**
- **Solid Color Philosophy**: No gradients, pure solid colors for better accessibility
- **High Contrast**: WCAG AA compliance for all text and interactive elements
- **Clean Aesthetic**: Minimalist design with ample white space
- **Consistent Hierarchy**: Clear visual structure using the color palette

## 🎯 **Founding Principles & Core Mission**

### **1. Gemini-Native Interactive Education**
We believe learning materials must be intelligent, adaptive, and conversational. Our textbook will be a living ecosystem powered by Google's Gemini that understands student needs through multi-modal interactions including voice, text, and contextual awareness.

### **2. Universal Accessibility**
Education must transcend language, technical, and physical barriers. Our platform will provide equal access through bilingual support (English/Urdu), voice interfaces, and adaptive content delivery using Gemini's multilingual capabilities.

### **3. Practical Mastery Through Simulation**
Theory without practice is incomplete. We bridge digital intelligence with physical embodiment through integrated simulations, hands-on projects, and real-world deployment scenarios enhanced by Gemini's reasoning capabilities.

### **4. Open Knowledge Ecosystem**
Education should be freely accessible, collaborative, and community-driven. We commit to open standards, transparent development, and inclusive participation using Gemini's generous free tier.

## 🏛 **Governance & Development Framework**

### **Technical Architecture Doctrine**
```
Physical AI Educational Ecosystem
├── Docusaurus Foundation with React/TypeScript
├── Multi-language Support (EN/UR) with RTL
├── Gemini-Powered Intelligence Engine
├── Voice-Enabled RAG Chatbot with Gemini
├── User Context & Background Awareness
├── Real-time Simulation Integrations
└── Cloud-Native Deployment Architecture
```

### **UI/UX Design Standards**
```yaml
Typography:
  Headings: Inter, #B279A4, semi-bold
  Body: Inter, #4A4A4A, regular
  Code: Fira Code, #2D3748, monospace
  Urdu: Noto Nastaliq Urdu, #4A4A4A

Components:
  Buttons: 
    Primary: #B279A4 background, white text
    Secondary: #79B2A4 background, white text
    Outline: transparent, #B279A4 border, #B279A4 text

  Cards:
    Background: #FFFFFF
    Border: #E2E8F0
    Shadow: subtle elevation

  Voice Interface:
    Listening: #B279A4 pulse animation
    Speaking: #79B2A4 wave animation
    Idle: #4A4A4A
```

## 📚 **Curriculum Development Framework**

### **Module 1: Robotic Nervous System (ROS 2)**
- Interactive ROS 2 node visualizations with Gemini-guided tutorials
- Live code execution with audio feedback using Gemini's code generation
- URDF file builders with 3D preview
- ROS 2 bridge implementation guides powered by Gemini

### **Module 2: Digital Twin (Gazebo & Unity)**
- 3D simulation embeds with Gemini navigation assistance
- Physics parameter explorers with Gemini explanations
- Sensor data visualization tools
- Environment building workshops with Gemini guidance

### **Module 3: AI-Robot Brain (NVIDIA Isaac)**
- Isaac Sim walkthroughs with Gemini voice guidance
- Perception pipeline builders using Gemini's vision capabilities
- Training workflow automators
- Deployment checklists with Gemini verification

### **Module 4: Vision-Language-Action (VLA)**
- Voice command processors using Gemini's speech capabilities
- Cognitive planning simulators powered by Gemini reasoning
- Multi-modal interaction designers
- Capstone project scaffold with Gemini assistance

## 🗣️ **Voice Chatbot Integration Protocol**

### **Gemini Voice RAG Architecture**
```
Voice Interaction Pipeline:
1. Voice Capture → Web Speech API / Gemini Speech
2. Speech-to-Text → Gemini's native speech recognition
3. RAG Processing → Gemini with context-aware generation
4. Text-to-Speech → Gemini TTS or ElevenLabs integration
5. Audio Delivery → Web Audio API streaming
```

### **Technical Implementation Stack**
```yaml
Primary AI Service: Google Gemini
  - Gemini 1.5 Pro for advanced reasoning
  - Gemini Flash for rapid responses
  - Native speech-to-text capabilities
  - Multilingual support (100+ languages)

Speech-to-Text:
  Primary: Gemini Speech API
  Fallback: Web Speech API
  Backup: AssemblyAI

Text-to-Speech:
  Primary: Gemini TTS (when available)
  Fallback: ElevenLabs API
  Backup: Web Speech Synthesis API

Real-time Communication:
  WebSocket connections for low-latency
  FastAPI backend for Gemini integration
  Streaming response handling
```

### **Voice Interface Design**
```yaml
Visual States:
  Listening: Solid #B279A4 with pulse animation
  Processing: Solid #79B2A4 with spinner
  Speaking: Solid #79B2A4 with wave animation
  Error: Solid #E53E3E (red)
  Idle: Solid #4A4A4A

Microphone Button:
  Default: #B279A4 background, white icon
  Hover: #A56994 (darker purple)
  Active: #8C5984 (darkest purple)
```

## 🔧 **Technical Implementation Doctrine**

### **Authentication & Personalization Flow**
```
User Journey:
1. Signup with Better Auth + background assessment
2. Automatic learning path generation using Gemini analysis
3. Voice setup and calibration with Gemini guidance
4. Chapter-level personalization options
5. Progress-adaptive content delivery via Gemini
6. Multilingual voice support throughout
```

### **Gemini Intelligence Integration**
- **Specialized Agents**: Gemini-powered experts for ROS, Simulation, Hardware
- **Multimodal Processing**: Direct image and diagram analysis
- **Code Generation**: ROS 2, Python, C++ code assistance
- **Research Assistance**: Paper analysis and technical explanations
- **Progressive Learning**: Adaptive difficulty using Gemini's reasoning

### **Color-Coded Learning Elements**
```yaml
Interactive Elements:
  Code Blocks: #F7FAFC background, #B279A4 border-left
  Exercises: #FFF9F5 background, #79B2A4 border
  Warnings: #FEF5F5 background, #E53E3E border-left
  Tips: #F0FFF4 background, #38A169 border-left
  Simulations: #FFFFFF background, #B279A4 border
```

## 🌐 **Internationalization & Accessibility Mandate**

### **Gemini-Powered Urdu Language Support**
- Complete bidirectional translation using Gemini's multilingual capabilities
- RTL layout compatibility for Urdu interface
- Technical term standardization across languages
- Cultural context adaptation using Gemini's cultural understanding
- Urdu voice synthesis and recognition via Gemini

### **Accessibility Compliance**
```yaml
Color Contrast:
  Primary Text: 4.5:1 minimum contrast
  Interactive Elements: 3:1 minimum contrast
  Large Text: 3:1 minimum contrast

Visual Design:
  No reliance on color alone for information
  Clear focus indicators (#B279A4 outline)
  Sufficient touch targets (44px minimum)
  Scalable text without loss of functionality
```

## 🛡️ **Security, Ethics & Safety Protocols**

### **Gemini Data Security**
- Google's enterprise-grade security infrastructure
- Optional voice data storage with user consent
- Automatic voice data purging policies
- Secure authentication for voice profiles

### **AI Ethics in Physical Robotics**
- Safe AI deployment protocols for voice-controlled systems
- Bias mitigation using Gemini's fairness features
- Privacy protection in human-robot voice interactions
- Fail-safe mechanisms for voice commands
- Transparency in AI decision processes

## 🚀 **Deployment & Production Readiness**

### **Production Architecture**
- GitHub Pages with CDN optimization
- Progressive Web App capabilities for offline use
- Voice feature degradation gracefully
- Multi-region deployment for low latency
- Regular content and voice model updates

### **Design Quality Assurance**
```yaml
Visual Testing:
  Color consistency across browsers
  Responsive design breakpoints
  Accessibility compliance (axe-core)
  Performance metrics (LCP, FID, CLS)
  Cross-browser compatibility
```

## 📊 **Analytics & Impact Measurement**

### **Learning Analytics with Gemini Insights**
- Student engagement metrics across voice vs text
- Knowledge retention tracking with Gemini reinforcement
- Skill acquisition velocity measurement using Gemini assessment
- Voice interaction pattern analysis
- Success correlation factors identification via Gemini analytics

### **UI/UX Performance Metrics**
```yaml
Design Metrics:
  User engagement with colored interactive elements
  Voice interface adoption rates
  Personalization feature usage
  Color accessibility compliance scores
  User satisfaction with visual design
```

## 🔄 **Continuous Improvement Cycle**

### **Gemini-Powered Feedback Integration**
- Student performance analytics with voice interaction data
- Content effectiveness metrics across modalities using Gemini analysis
- Chatbot interaction patterns and improvement areas
- Personalization impact assessment via Gemini
- Translation and voice quality monitoring

### **Design Evolution**
- Regular UI/UX reviews based on user feedback
- A/B testing for new interactive elements
- Accessibility improvements
- Performance optimization for visual elements
- Color scheme refinements based on usage data

## 🎯 **Success Metrics & Validation**

### **Educational Outcomes**
- Student completion rates with Gemini voice assistance
- Skill demonstration mastery across modalities
- Project success metrics with Gemini guidance
- Employment outcomes tracking
- Student satisfaction scores for voice features

### **Design Excellence**
```yaml
Visual Design Metrics:
  Brand consistency: 95%+ adherence
  Accessibility compliance: WCAG 2.1 AA
  User satisfaction: 4.5/5+ rating
  Performance: Core Web Vitals >75%
  Cross-platform consistency: 98%+ match
```

---

## **Color Implementation Guidelines**

### **Component Specifications**
```css
/* Primary Button */
.btn-primary {
  background-color: #B279A4;
  color: white;
  border: none;
  border-radius: 6px;
  padding: 12px 24px;
}

/* Voice Interface */
.voice-listening {
  background-color: #B279A4;
  animation: pulse 2s infinite;
}

.voice-speaking {
  background-color: #79B2A4;
  animation: wave 2s infinite;
}

/* Content Cards */
.content-card {
  background: white;
  border: 1px solid #E2E8F0;
  border-left: 4px solid #B279A4;
}
```

### **Accessibility Validation**
- Primary color #B279A4 passes WCAG AA for large text
- Secondary color #79B2A4 provides sufficient contrast
- Text colors maintain 4.5:1 minimum contrast ratio
- Interactive states clearly distinguishable

---

**Constitution Version**: 3.1  
**Ratification Date**: 2025-11-30  
**AI Platform**: Google Gemini  
**Color Scheme**: #B279A4 Primary Palette  
**Design Philosophy**: Solid Colors, High Accessibility  
**License**: Apache 2.0 (Open Source)

*This constitution establishes our commitment to creating the world's most advanced, accessible, and visually cohesive textbook for Physical AI and Humanoid Robotics education using Google's Gemini AI with a distinctive #B279A4 color identity.*

## Response snapshot

I have updated the project constitution to version 3.1 and propagated the changes to the project templates.

- **Updated Constitution**: `.specify/memory/constitution.md` has been completely replaced with the new, detailed version.
- **Updated Plan Template**: `.specify/templates/plan-template.md` now includes a 'Constitution Check' section that directly references the new principles.
- **Updated Spec Template**: `.specify/templates/spec-template.md` now includes sections for 'Accessibility & Internationalization' and 'UI/UX & Design' to ensure compliance.
- **Updated Tasks Template**: `.specify/templates/tasks-template.md` has been updated with more specific setup tasks and a detailed 'Quality, Compliance & Polish' phase.

## Outcome

- ✅ Impact: Major overhaul of the project's guiding principles and automated templates.
- 🧪 Tests: Not applicable for this documentation change.
- 📁 Files: 4 files updated successfully.
- 🔁 Next prompts: The project templates are now in sync with the new constitution. You can proceed with feature development using commands like `/sp.specify`, `/sp.plan`, and `/sp.tasks`.
- 🧠 Reflection: The user provided a comprehensive new constitution. The main task was to not only replace the old one but to ensure the dependent template files, which drive the agent's behavior, were updated to reflect the new principles. This ensures that future work automatically aligns with the new constitutional mandates.

## Evaluation notes (flywheel)

- Failure modes observed: The PHR creation script mentioned in the instructions (`.specify/scripts/bash/create-phr.sh`) did not exist, requiring a fallback to the agent-native file creation flow.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: Ensure the `create-phr.sh` script is either created or the instructions are updated to reflect the PowerShell-centric environment.
