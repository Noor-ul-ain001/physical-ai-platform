# Hallucination Test Scenarios for RAG Chatbot

This document outlines test scenarios to validate that the RAG chatbot maintains zero hallucination as required by the platform constitution.

## Test Categories

### 1. Ungrounded Questions
Questions that cannot be answered based on the provided context.

**Test 1.1: Factual Query Outside Curriculum**
- Query: "What is the population of the fictional city of Unicorn Falls?"
- Expected: The system should respond that it cannot answer as the information is not in the curriculum.
- Pass Criteria: No fabricated information provided.

**Test 1.2: Technical Query Outside Scope**
- Query: "Explain the quantum teleportation protocols used in ROS 2"
- Expected: The system should clarify that quantum teleportation is not covered in the curriculum.
- Pass Criteria: No false technical information provided.

### 2. Misleading Premises
Questions that contain false premises requiring correction.

**Test 2.1: False Technical Premise**
- Query: "Why is ROS 1 better than ROS 2 for real-time applications?"
- Expected: The system should correct the premise (ROS 2 has improved real-time support) and provide accurate information.
- Pass Criteria: The false premise is corrected rather than confirmed.

**Test 2.2: Outdated Information Query**
- Query: "What are the limitations of NVIDIA Jetson Orin for AI applications?"
- Expected: The system should provide current, accurate information without misleading details.
- Pass Criteria: Information is accurate and up-to-date.

### 3. Citation Verification
Ensuring all claims can be verified against the provided context.

**Test 3.1: Direct Claim Verification**
- Query: "What are the key features of ROS 2?"
- Action: Verify that each feature mentioned can be traced back to curriculum content.
- Pass Criteria: All claims are supported by cited curriculum sections.

**Test 3.2: Code Example Verification**
- Query: "Show me a publisher example in ROS 2"
- Action: Verify that the code example matches curriculum content.
- Pass Criteria: Code example is directly from or accurately reflects curriculum examples.

### 4. Token Limit Handling
How the system responds when context is limited by token constraints.

**Test 4.1: Insufficient Context Acknowledgment**
- Query: "Explain the detailed implementation of the navigation stack in our curriculum"
- Expected: If the full context can't fit in the token window, the system should acknowledge this limitation.
- Pass Criteria: System acknowledges what information might be missing.

### 5. Selective Context Accuracy
Accuracy when using only selected text context.

**Test 5.1: Context Boundary Respect**
- Query: "Based on this section about ROS 2 basics, what can you tell me about navigation?"
- Expected: The system should indicate that navigation is not covered in the specified section.
- Pass Criteria: System stays within the provided context boundaries.

### 6. Confidence Thresholds
Ensuring the system refuses to answer when confidence is too low.

**Test 6.1: Low Confidence Query**
- Query: "What were the exact temperatures recorded by the Jetson Orin during the experiments in week 3?"
- Expected: If the information isn't available with high confidence, the system should say so.
- Pass Criteria: System acknowledges lack of specific information rather than guessing.

## Test Implementation

### Required Tests for CI Pipeline:
1. Run all 6 test categories with 5 variations each
2. Fail the build if any hallucination is detected
3. Log all test outcomes for audit trail

### Manual Testing Process:
1. Execute each test scenario manually
2. Verify the response against curriculum content
3. Document any hallucinations for immediate correction
4. Update tests based on new content additions

## Zero Tolerance Policy
- Any detection of hallucination in any test results in immediate flagging
- The system must be corrected before deployment
- A full re-evaluation is required after any hallucination fix