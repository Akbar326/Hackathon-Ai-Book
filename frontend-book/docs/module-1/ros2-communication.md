---
sidebar_label: ROS 2 Communication Primitives
title: ROS 2 Communication Primitives
---

# ROS 2 Communication Primitives

## Learning Objectives

By the end of this chapter, you will be able to:
- Identify and differentiate between Nodes, Topics, Publishers, and Subscribers
- Understand Services and Actions (conceptual overview)
- Describe data flow between perception, planning, and control layers
- Apply appropriate communication patterns to different robot system scenarios

## ROS 2 Nodes and Executors

### What is a Node?

A node is the fundamental unit of computation in ROS 2. It's a process that performs computation and communicates with other nodes. Nodes can be written in different programming languages (C++, Python, etc.) and can run on different machines while still being able to communicate with each other.

### Creating a Node

In ROS 2, nodes are typically created by inheriting from the `rclpy.Node` class (in Python) or `rclcpp::Node` class (in C++). Each node must have a unique name within the ROS 2 domain.

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        # Node initialization code here
```

### Executors

An executor manages the execution of one or more nodes. It handles the scheduling of callbacks and ensures that nodes can process incoming messages and perform their computations. The most common executor is the `SingleThreadedExecutor`, but ROS 2 also supports `MultiThreadedExecutor` for more complex scenarios.

## Topics, Publishers, and Subscribers

### Topics

Topics are named buses over which nodes exchange messages. They implement a publish-subscribe communication pattern, which is asynchronous and loosely coupled. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic.

### Publishers

A publisher is a node that sends messages to a topic. Publishers are created within nodes and specify the topic name and message type.

```python
from std_msgs.msg import String

publisher = self.create_publisher(String, 'topic_name', 10)
```

The third parameter is the queue size, which determines how many messages can be queued for delivery if subscribers are not ready to receive them.

### Subscribers

A subscriber is a node that receives messages from a topic. Subscribers are also created within nodes and specify the topic name, message type, and callback function.

```python
def topic_callback(self, msg):
    self.get_logger().info('Received: %s' % msg.data)

subscriber = self.create_subscription(
    String,
    'topic_name',
    self.topic_callback,
    10)
```

### Publish-Subscribe Pattern Characteristics

- **Loose Coupling**: Publishers and subscribers don't need to know about each other
- **Asynchronous**: Publishers don't wait for responses from subscribers
- **Broadcast**: One publisher can send to multiple subscribers
- **Decoupled Timing**: Publishers and subscribers can come and go without affecting each other

## Services and Actions (Conceptual Overview)

### Services

Services implement a request-response communication pattern, which is synchronous and tightly coupled. A client sends a request to a service and waits for a response. This pattern is suitable for tasks that have a clear beginning and end.

```python
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    return response

service = self.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
```

### Actions

Actions are designed for long-running tasks that require feedback and the ability to cancel. They combine aspects of both topics and services, providing:
- Goal requests (like service requests)
- Continuous feedback during execution
- Result responses (like service responses)
- Cancellation capability

Actions are particularly useful for tasks like:
- Moving a robot to a specific location
- Performing a complex manipulation task
- Running a calibration procedure

## Data Flow Between Perception, Planning, and Control Layers

### Perception Layer

The perception layer is responsible for processing sensor data to understand the environment and the robot's state. Common communication patterns in the perception layer include:

**Sensor Data Topics**:
- Camera images (`sensor_msgs/Image`)
- Point clouds (`sensor_msgs/PointCloud2`)
- Laser scan data (`sensor_msgs/LaserScan`)
- IMU data (`sensor_msgs/Imu`)

**Processed Perception Results**:
- Detected objects (`vision_msgs/Detection2DArray`)
- Occupancy grids (`nav_msgs/OccupancyGrid`)
- Robot pose estimates (`geometry_msgs/PoseStamped`)

### Planning Layer

The planning layer takes information from perception and generates plans for robot behavior. Communication patterns include:

**Planning Requests**:
- Navigation goals (`geometry_msgs/PoseStamped`)
- Path planning requests (`nav_msgs/Path`)

**Plan Results**:
- Generated paths (`nav_msgs/Path`)
- Trajectory plans (`trajectory_msgs/JointTrajectory`)

### Control Layer

The control layer executes the plans generated by the planning layer. Communication patterns include:

**Control Commands**:
- Joint position/velocity commands (`std_msgs/Float64MultiArray`)
- Velocity commands (`geometry_msgs/Twist`)

**Feedback**:
- Joint states (`sensor_msgs/JointState`)
- Odometry (`nav_msgs/Odometry`)

### Example Data Flow

Here's a typical data flow in a robot system:

1. **Perception**: Camera nodes publish images to `/camera/image_raw`
2. **Perception**: Object detection nodes subscribe to images and publish detections to `/object_detections`
3. **Planning**: Navigation planner subscribes to detections and map data to plan a path to `/path`
4. **Control**: Controller subscribes to path and publishes velocity commands to `/cmd_vel`
5. **Control**: Motor controllers execute commands and publish feedback via `/joint_states`

## Best Practices for Communication Design

### Choosing the Right Communication Pattern

- **Use Topics** for continuous data streams (sensor data, robot states)
- **Use Services** for discrete, stateless operations (setting parameters, simple computations)
- **Use Actions** for long-running, cancellable tasks (navigation, manipulation)

### Quality of Service (QoS) Considerations

ROS 2 provides Quality of Service settings that allow you to tune communication behavior:

- **Reliability**: Reliable (delivery guaranteed) vs Best Effort (delivery not guaranteed)
- **Durability**: Transient Local (messages stored for late-joining subscribers) vs Volatile
- **History**: Keep Last N messages vs Keep All messages

### Naming Conventions

- Use forward slashes to separate namespaces: `/robot1/sensors/camera`
- Use descriptive names that indicate the content: `/joint_states` vs `/js`
- Follow established conventions for common message types

## Summary

ROS 2 communication primitives provide the foundation for building complex robotic systems. Understanding when and how to use nodes, topics, publishers, subscribers, services, and actions is crucial for creating well-architected robot software. The publish-subscribe pattern enables loose coupling and scalability, while services and actions provide synchronous and long-running communication patterns as needed.

## Exercises and Thought Experiments

1. **Scenario Analysis**: For each of the following scenarios, identify the most appropriate communication pattern (Topic, Service, or Action) and explain why:
   - Requesting the robot's current battery level
   - Streaming camera images from a robot's head camera
   - Asking the robot to navigate to a specific location
   - Updating the robot's configuration parameters

2. **System Design**: Design a simple robot system with 3-4 nodes that would enable a robot to detect and approach a colored ball. Specify the topics, services, or actions that would be used for communication between nodes.

3. **QoS Considerations**: For the robot system you designed above, which Quality of Service settings would be most appropriate for each communication link? Consider reliability, durability, and history requirements.

## Next Steps

- Review [Introduction to ROS 2 for Physical AI](./intro) if you need to refresh the foundational concepts
- Continue to [Bridging AI Agents with ROS 2](./ai-to-robot-bridge) to learn how to connect your AI algorithms with these communication primitives