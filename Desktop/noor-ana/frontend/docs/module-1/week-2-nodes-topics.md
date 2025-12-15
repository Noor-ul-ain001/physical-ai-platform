---
sidebar_position: 2
---

# Week 2: Nodes and Topics

## Overview

This week focuses on two fundamental concepts in ROS 2: Nodes and Topics. Nodes are the basic execution units of a ROS 2 program, and topics are the buses over which nodes exchange messages.

## Learning Objectives

By the end of this week, you will:
- Understand the concept of nodes in ROS 2
- Create and run ROS 2 nodes
- Implement topic-based communication
- Use publishers and subscribers effectively
- Debug node communication issues

## Nodes in ROS 2

A node is a collection of processes that perform computation. In ROS 2, nodes are implemented as separate processes that can run on the same or different machines. Nodes communicate with each other using topics, services, and actions.

### Creating a Node

Here's the basic structure of a ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code goes here

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Messages

Topics are named buses over which nodes exchange messages. Messages are data structures that are passed between nodes. ROS 2 provides several built-in message types and allows users to define custom messages.

### Publisher and Subscriber

```python
# Publisher example
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

# Subscriber example
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

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
```

## Advanced Topic Concepts

### Quality of Service (QoS)

ROS 2 provides Quality of Service (QoS) settings that allow fine-tuning of communication behavior:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Create a custom QoS profile
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE
)

# Use it when creating a publisher
publisher = node.create_publisher(String, 'topic', qos_profile)
```

### Topic Commands

Useful command-line tools for working with topics:

```bash
# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic_name

# Print information about a topic
ros2 topic info /topic_name

# Publish a message to a topic
ros2 topic pub /topic_name std_msgs/String "data: 'Hello World'"
```

## Practical Example: Temperature Sensor

Let's implement a complete example with a temperature sensor that publishes readings and a monitor that subscribes to those readings:

```python
# temperature_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Float32()
        # Simulate temperature reading (20-30 degrees Celsius)
        msg.data = 20.0 + random.uniform(0, 10)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data:.2f}°C')

# temperature_subscriber.py
class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)
        self.subscription  # prevent unused variable warning

    def temperature_callback(self, msg):
        temperature = msg.data
        if temperature > 25.0:
            self.get_logger().warn(f'High temperature detected: {temperature:.2f}°C')
        else:
            self.get_logger().info(f'Temperature: {temperature:.2f}°C')
```

## Debugging Tips

1. Use `ros2 topic list` to verify your topics exist
2. Use `ros2 topic echo /topic_name` to see if messages are being published
3. Check node logs with `ros2 run --prefix 'gdb -ex run --args' package_name node_name` for debugging
4. Use `rqt_graph` to visualize the node communication graph

## Hardware Considerations

When working with actual hardware nodes:
- Ensure network connectivity between nodes
- Consider bandwidth limitations for sensor data
- Account for latency in real-time applications
- Implement appropriate error handling for hardware communication

## Summary

This week covered nodes and topics, the fundamental building blocks of ROS 2 communication. You learned how to create nodes, implement publishers and subscribers, and work with different message types. Next week, we'll explore services and actions for different communication patterns.

## Exercises

1. Create a node that publishes the current system time
2. Create a subscriber that logs any published timestamps
3. Implement a QoS configuration that ensures durability for critical messages
4. Debug a provided node with a communication issue