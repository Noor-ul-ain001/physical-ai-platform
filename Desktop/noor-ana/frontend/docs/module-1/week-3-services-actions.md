---
sidebar_position: 3
---

# Week 3: Services and Actions

## Overview

This week focuses on two more communication patterns in ROS 2: Services and Actions. While topics provide asynchronous, decoupled communication, services and actions provide synchronous and goal-oriented communication respectively.

## Learning Objectives

By the end of this week, you will:
- Understand the difference between topics, services, and actions
- Implement ROS 2 services (request/response)
- Implement ROS 2 actions (goal/feedback/result)
- Choose the appropriate communication pattern for different use cases
- Debug service and action communication

## Services in ROS 2

Services implement a request/reply communication pattern. A service client sends a request to a service server, which processes the request and returns a response. This is synchronous communication.

### Service Definition

Service definitions are stored in `.srv` files with two parts: the request and the response, separated by three dashes (`---`).

Example: `AddTwoInts.srv`
```
int64 a
int64 b
---
int64 sum
```

### Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(41, 1)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions in ROS 2

Actions are used for long-running tasks that provide feedback and can be canceled. They follow a goal/feedback/result pattern and are ideal for tasks like navigation or manipulation.

### Action Definition

Action definitions are stored in `.action` files with three parts: Goal, Result, and Feedback.

Example: `Fibonacci.action`
```
int32 order
---
int32[] sequence
---
int32[] sequence
```

### Action Server

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class MinimalActionServer(Node):
    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal was canceled')
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = MinimalActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class MinimalActionClient(Node):
    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = MinimalActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Choosing the Right Communication Pattern

| Pattern | Use Case | Characteristics |
|---------|----------|-----------------|
| Topics | Continuous data streams | Asynchronous, decoupled |
| Services | Simple request/response | Synchronous, blocking |
| Actions | Long-running tasks | Asynchronous, with feedback and cancellation |

### When to Use Each Pattern

- **Topics**: Sensor data streams, state publishing, status updates
- **Services**: Simple computations, configuration changes, one-time requests
- **Actions**: Navigation goals, manipulation tasks, calibration sequences

## Practical Example: Robot Arm Controller

Let's implement a comprehensive example using all three communication patterns:

```python
# Robot Arm Controller
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from example_interfaces.srv import SetBool
from example_interfaces.action import MultiGoal
from rclpy.action import ActionServer, GoalResponse

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # Topic: Joint angles feedback
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.joint_publisher = self.create_publisher(Float32MultiArray, 'joint_angles', qos_profile)
        
        # Service: Check if arm is ready
        self.ready_service = self.create_service(SetBool, 'arm_ready', self.arm_ready_callback)
        
        # Action: Move to position
        self.move_action_server = ActionServer(
            self,
            MultiGoal,
            'move_arm',
            self.execute_move_callback
        )
        
        # Simulate joint angles
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 joints
        self.timer = self.create_timer(0.1, self.publish_joint_angles)
    
    def publish_joint_angles(self):
        msg = Float32MultiArray()
        msg.data = self.joint_angles
        self.joint_publisher.publish(msg)
    
    def arm_ready_callback(self, request, response):
        # Check if all joints are within safe limits
        is_safe = all(-3.14 <= angle <= 3.14 for angle in self.joint_angles)
        response.success = is_safe
        response.message = f'Arm is {"ready" if is_safe else "not ready"}'
        return response
    
    def execute_move_callback(self, goal_handle):
        self.get_logger().info(f'Executing move to: {goal_handle.request.goals}')
        # Simulate movement
        self.joint_angles = goal_handle.request.goals
        goal_handle.succeed()
        result = MultiGoal.Result()
        result.result = True
        return result

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service and Action Commands

Useful command-line tools for working with services and actions:

```bash
# Services
ros2 service list                    # List all services
ros2 service info /service_name      # Get information about a service
ros2 service type /service_name      # Get service type
ros2 service call /service_name service_type "request_data"

# Actions
ros2 action list                     # List all actions
ros2 action info /action_name        # Get information about an action
ros2 action send_goal /action_name action_type "goal_data"
```

## Best Practices

1. Use services for simple, quick operations that have a clear result
2. Use actions for long-running operations that may need cancellation
3. Use topics for continuous data streams
4. Always handle service and action client errors appropriately
5. Implement proper timeouts for service calls

## Summary

This week covered services and actions, two important communication patterns in ROS 2. Services provide synchronous request/response communication, while actions are designed for long-running tasks with feedback and cancellation. You learned how to implement and use both patterns, and when to apply each one. 

Next week, we'll explore TF (Transforms) and URDF (Unified Robot Description Format), which are essential for spatial relationships and robot modeling.

## Exercises

1. Create a service that calculates the Euclidean distance between two points
2. Implement an action that performs a 10-second counting operation with feedback
3. Build a robot simulator node that publishes transform data
4. Debug a provided service implementation with incorrect response handling