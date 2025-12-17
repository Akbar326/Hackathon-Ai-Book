---
sidebar_label: Bridging AI Agents with ROS 2
title: From AI Agents to Robot Bodies
---

# From AI Agents to Robot Bodies

## Learning Objectives

By the end of this chapter, you will be able to:
- Connect Python AI agents with ROS 2 using rclpy
- Differentiate between high-level control and low-level motor commands
- Understand the basics of URDF for humanoid robots
- Explain how robot structure enables physical embodiment

## Bridging Python AI Agents with ROS 2 using rclpy

### Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides the Python API that allows Python programs to interact with the ROS 2 middleware. This is the essential bridge that connects your Python-based AI agents with the ROS 2 ecosystem.

### Basic rclpy Setup

To use rclpy in your Python AI agent, you need to:

1. Initialize the ROS client library
2. Create a node that will contain your AI logic
3. Create publishers, subscribers, services, or actions as needed
4. Spin the node to process callbacks
5. Clean up when done

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Create a publisher to send commands to the robot
        self.publisher = self.create_publisher(String, 'ai_commands', 10)

        # Create a subscriber to receive sensor data from the robot
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10)

        # Timer to run AI logic periodically
        self.timer = self.create_timer(0.1, self.ai_logic)

    def sensor_callback(self, msg):
        # Process sensor data from the robot
        self.get_logger().info(f'Received sensor data: {msg.data}')
        # Update AI agent state based on sensor data

    def ai_logic(self):
        # Your AI algorithm runs here
        # Based on current state, decide what the robot should do
        command = String()
        command.data = 'AI decision output'
        self.publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integration Patterns

When bridging AI agents with ROS 2, several patterns are commonly used:

#### 1. Perception-Action Loop

This pattern continuously receives sensor data, processes it through AI algorithms, and sends actions to the robot:

```python
class PerceptionActionNode(Node):
    def __init__(self):
        super().__init__('perception_action_node')

        # Subscribe to sensor data
        self.sensor_sub = self.create_subscription(
            Image,  # or LaserScan, PointCloud2, etc.
            'camera/image_raw',
            self.process_perception,
            10)

        # Publish actions to robot
        self.action_pub = self.create_publisher(
            Twist,  # or JointState, etc.
            'cmd_vel',
            10)

    def process_perception(self, sensor_msg):
        # Convert ROS message to format suitable for AI
        ai_input = self.convert_sensor_to_ai_format(sensor_msg)

        # Run AI algorithm
        ai_output = self.run_ai_algorithm(ai_input)

        # Convert AI output to ROS message
        action_msg = self.convert_ai_to_action_format(ai_output)

        # Publish action to robot
        self.action_pub.publish(action_msg)
```

#### 2. Planning and Execution

This pattern separates high-level planning from low-level execution:

```python
class PlanningExecutionNode(Node):
    def __init__(self):
        super().__init__('planning_execution_node')

        # Subscribe to goals and sensor data
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal',
            self.plan_trajectory,
            10)

        # Publish planned trajectories
        self.trajectory_pub = self.create_publisher(
            Path,
            'planned_path',
            10)

    def plan_trajectory(self, goal_msg):
        # Use AI planning algorithm to generate trajectory
        path = self.ai_planning_algorithm(goal_msg)

        # Publish planned path for execution
        self.trajectory_pub.publish(path)
```

### Best Practices for AI-ROS Integration

1. **Asynchronous Processing**: Use separate threads or async patterns for heavy AI computation to avoid blocking ROS message processing
2. **Message Conversion**: Create helper functions to convert between ROS message formats and AI-friendly formats
3. **Error Handling**: Implement robust error handling for when AI algorithms fail or produce unexpected outputs
4. **Performance Monitoring**: Monitor the performance of AI algorithms to ensure real-time constraints are met

## High-Level Control vs Low-Level Motor Commands

### Control Hierarchy in Robotics

Robot control is typically organized in a hierarchical structure, with different levels of abstraction:

#### Level 1: High-Level AI Control

- **Responsibility**: Decision making, planning, reasoning
- **Input**: High-level goals, environment information
- **Output**: Abstract commands or plans
- **Example**: "Navigate to the kitchen", "Pick up the red object"

#### Level 2: Path/Motion Planning

- **Responsibility**: Convert high-level goals to specific trajectories
- **Input**: High-level commands, environment maps
- **Output**: Trajectories or paths
- **Example**: Specific path coordinates, velocity profiles

#### Level 3: Low-Level Motor Control

- **Responsibility**: Execute specific motor commands
- **Input**: Trajectory points, desired positions/velocities
- **Output**: Direct motor commands (PWM, currents, torques)
- **Example**: Joint angles, motor speeds

### Communication Between Control Levels

ROS 2 facilitates communication between these control levels using appropriate message types:

```python
# High-level command (navigation goal)
from geometry_msgs.msg import PoseStamped

# Path planning output
from nav_msgs.msg import Path

# Low-level motor commands
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
```

### Example: Multi-Level Control Architecture

```python
class HighLevelController(Node):
    def __init__(self):
        super().__init__('high_level_controller')

        # Send high-level goals to planner
        self.goal_publisher = self.create_publisher(
            PoseStamped, 'navigation_goal', 10)

        # Receive status from lower levels
        self.status_subscriber = self.create_subscription(
            String, 'system_status', self.status_callback, 10)

    def ai_decision_making(self, environment_state):
        # AI algorithm decides what the robot should do
        goal = self.determine_navigation_goal(environment_state)
        self.goal_publisher.publish(goal)

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        # Receive goals from high level
        self.goal_subscriber = self.create_subscription(
            PoseStamped, 'navigation_goal', self.plan_path, 10)

        # Send paths to low-level controller
        self.path_publisher = self.create_publisher(
            Path, 'planned_path', 10)

    def plan_path(self, goal):
        # Plan a collision-free path
        path = self.path_planning_algorithm(goal)
        self.path_publisher.publish(path)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Receive trajectories from planner
        self.path_subscriber = self.create_subscription(
            Path, 'planned_path', self.execute_trajectory, 10)

        # Send commands to motors
        self.motor_publisher = self.create_publisher(
            JointState, 'motor_commands', 10)

    def execute_trajectory(self, path):
        # Convert path to motor commands
        motor_commands = self.trajectory_to_motors(path)
        self.motor_publisher.publish(motor_commands)
```

## Introduction to URDF for Humanoid Robots

### What is URDF?

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It specifies the physical properties of a robot, including:

- **Links**: Rigid parts of the robot (e.g., torso, arms, legs)
- **Joints**: Connections between links (e.g., hinges, prismatic joints)
- **Inertial properties**: Mass, center of mass, and inertia tensors
- **Visual properties**: How the robot appears in simulation
- **Collision properties**: How the robot interacts with the environment

### URDF Structure for Humanoid Robots

A humanoid robot URDF typically includes:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.5 0.3 0.8" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.5 0.3 0.8" />
      </geometry>
    </collision>
  </link>

  <!-- Example joint connecting base to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link" />
    <child link="head" />
    <origin xyz="0.0 0.0 0.6" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0" />
  </joint>

  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.15" />
      </geometry>
    </visual>
  </link>
</robot>
```

### Key Components of Humanoid URDF

#### Links
- **Torso/Body**: The main body of the robot
- **Head**: Contains cameras, sensors
- **Arms**: Each with shoulder, elbow, and wrist joints
- **Legs**: Each with hip, knee, and ankle joints
- **Feet**: For balance and ground contact

#### Joints
- **Revolute**: Rotational joints (like human joints)
- **Fixed**: Non-moving connections
- **Prismatic**: Linear sliding joints
- **Continuous**: Joints that can rotate indefinitely

#### Sensors
URDF can also include sensor specifications:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

### Working with URDF in ROS 2

To use URDF models in ROS 2:

1. **Launch the robot state publisher**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
            ]
        )
    ])
```

2. **Publish joint states** to see the robot in RViz:
```python
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
```

## How Robot Structure Enables Physical Embodiment

### Physical Embodiment in Robotics

Physical embodiment refers to the idea that the physical form and properties of a robot are integral to its intelligence and behavior. The robot's structure directly affects:

- What it can perceive about the environment
- What actions it can perform
- How it can interact with objects
- How it can move and navigate

### Structure-Function Relationship

The physical structure of a humanoid robot determines its capabilities:

#### Locomotion Capabilities
- **Bipedal structure**: Enables human-like walking but requires sophisticated balance control
- **Joint ranges**: Determine the robot's range of motion and agility
- **Mass distribution**: Affects stability and energy efficiency

#### Manipulation Capabilities
- **Number of DOF (Degrees of Freedom)**: More joints allow for more complex manipulation
- **End effector design**: Hands, grippers, or tools determine what objects can be manipulated
- **Workspace**: The reachable volume determines what the robot can interact with

#### Sensing Capabilities
- **Sensor placement**: Cameras, IMUs, and other sensors positioned on the robot body
- **Viewpoint**: The robot's height and sensor positions affect what it can perceive
- **Embodied sensing**: How the robot's movement affects its sensing (active perception)

### AI Adaptation to Physical Structure

AI agents must be designed to work with the specific physical embodiment of the robot:

```python
class EmbodiedAINode(Node):
    def __init__(self):
        super().__init__('embodied_ai')

        # Subscribe to sensor data from the robot's embodiment
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.process_balance, 10)

        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.process_posture, 10)

        # The AI algorithms must consider the robot's physical constraints
        self.robot_kinematics = self.load_robot_urdf()  # Load from URDF
        self.physical_constraints = self.extract_constraints(self.robot_kinematics)

    def process_balance(self, imu_msg):
        # Use AI to maintain balance based on IMU data
        # The AI knows the robot is bipedal from its URDF
        if self.detecting_instability(imu_msg):
            self.generate_balance_recovery_action()

    def generate_balance_recovery_action(self):
        # Generate actions appropriate for the robot's structure
        # e.g., shift weight, take a step, adjust arm position
        action = self.balance_controller.compute_stabilizing_action()
        self.publish_action(action)
```

### Embodied AI Principles

When designing AI for embodied robots:

1. **Embodiment Awareness**: The AI should understand the robot's physical form and limitations
2. **Sensorimotor Integration**: Perception and action are tightly coupled
3. **Morphological Computation**: Use the robot's physical properties to simplify control
4. **Active Perception**: The AI can control where and how sensors are positioned

## Summary

Bridging Python AI agents with ROS 2 using rclpy enables the integration of sophisticated artificial intelligence with physical robot systems. Understanding the hierarchy between high-level control and low-level motor commands is crucial for creating effective robot behaviors. URDF provides the essential description of robot structure that enables both simulation and real-world interaction. The physical embodiment of a robot directly shapes its capabilities and must be considered when designing AI algorithms for robotic systems.

## Exercises and Thought Experiments

1. **Implementation Challenge**: Write a Python script that creates an rclpy node that subscribes to a camera topic and publishes velocity commands to make a robot turn toward the brightest area in the camera image. What communication patterns would you use?

2. **URDF Analysis**: Consider a simple wheeled robot with a camera. Draw or describe the URDF structure it would need. What links and joints would be required? What sensors would be important to include?

3. **Control Hierarchy Design**: Design a control hierarchy for a robot that needs to pick up objects from a table. Identify what would happen at the high-level AI control, path/motion planning, and low-level motor control levels.

## Next Steps

- Review [Introduction to ROS 2 for Physical AI](./intro) to refresh foundational concepts
- Revisit [ROS 2 Communication Primitives](./ros2-communication) to strengthen your understanding of communication patterns
- This completes our module on ROS 2 as the robotic nervous system. You now have the foundation to understand how AI agents can interact with and control physical robot systems.