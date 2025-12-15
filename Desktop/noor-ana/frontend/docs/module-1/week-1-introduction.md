---
sidebar_position: 1
---

# Week 1: Introduction to ROS 2

## Overview

This week introduces the Robot Operating System 2 (ROS 2), a flexible framework for writing robot software. ROS 2 is not a real-time operating system but a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## Learning Objectives

By the end of this week, you will:
- Understand what ROS 2 is and its role in robotics
- Know the key differences between ROS 1 and ROS 2
- Be familiar with the ROS 2 architecture
- Install and set up ROS 2 on your development environment

## What is ROS 2?

ROS 2 is the second generation of the Robot Operating System, an open-source framework for developing robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS 2 improves upon the original ROS with better security, real-time support, and improved build system among other enhancements.

### Key Features
- Distributed computing framework for building robotic applications
- Platform for developing sensor processing, mapping, and planning applications
- Set of conventions for creating reusable robot software components
- Active community and extensive documentation

## ROS 1 vs. ROS 2

ROS 2 was developed to address several limitations of ROS 1:

| Feature | ROS 1 | ROS 2 |
|--------|--------|--------|
| Communication | Custom TCP/UDP | DDS (Data Distribution Service) |
| Security | No built-in security | Comprehensive security model |
| Real-time support | Limited | Native real-time support |
| Multi-robot support | Challenging | Improved |
| Build system | catkin | ament (CMake-based) |
| OS support | Linux/macOS primarily | Linux, Windows, macOS, RTOS |

## ROS 2 Architecture

ROS 2 uses a DDS (Data Distribution Service) implementation for communication between processes. DDS is a standard communication middleware for real-time, distributed systems.

### Core Concepts

1. **Nodes**: Processes that perform computation. Nodes are the fundamental unit of computation in ROS 2.
2. **Messages**: Data structures used for communication between nodes.
3. **Topics**: Names to which messages are sent and received. Publishers send messages, subscribers receive messages.
4. **Services**: Synchronous request/response communication pattern.
5. **Actions**: Asynchronous goal-oriented communication pattern.

```python
# Example ROS 2 Node
import rclpy
from rclpy.node import Node

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
```

## Installation

We'll use the official ROS 2 distribution for your platform. The most recent stable distribution is Humble Hawksbill, which has long-term support (LTS).

For detailed installation instructions, refer to the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

### Hardware Requirements for This Week

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4 cores | 8+ cores |
| RAM | 8 GB | 16+ GB |
| GPU | Integrated | RTX 3060+ |
| Storage | 50 GB | 100+ GB |

## Summary

This week provided an overview of ROS 2, its architecture, and installation process. Next week, we'll dive into working with nodes and topics, the fundamental communication mechanisms in ROS 2.

## Exercises

1. Install ROS 2 Humble Hawksbill on your development machine
2. Run the basic publisher/subscriber example
3. Explore the ROS 2 command-line tools (ros2 run, ros2 topic, etc.)