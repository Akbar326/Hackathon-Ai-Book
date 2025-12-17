---
sidebar_label: Nav2 Navigation
title: Nav2 for Humanoid Navigation
---

# Nav2 for Humanoid Navigation

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure and implement path planning and obstacle avoidance for humanoid robots
- Set up navigation stacks specifically for bipedal humanoids
- Integrate Nav2 with ROS 2 control layers for humanoid locomotion

## Introduction to Nav2 for Humanoid Robots

Navigation2 (Nav2) is ROS 2's state-of-the-art navigation stack, designed for autonomous robot navigation. While originally developed for wheeled robots, Nav2 can be adapted for humanoid robots with specific configurations that account for the unique challenges of bipedal locomotion.

### Nav2 Architecture Overview

Nav2 follows a behavior tree-based architecture that allows for flexible and robust navigation:

- **Navigation Server**: Central component that coordinates navigation tasks
- **Behavior Trees**: Define the logic for navigation behaviors
- **Planners**: Global and local path planning components
- **Controllers**: Trajectory generation and execution
- **Recovery Behaviors**: Actions to take when navigation fails

### Humanoid-Specific Navigation Challenges

Humanoid robots present unique challenges for navigation:

- **Bipedal Kinematics**: Different motion patterns compared to wheeled robots
- **Balance Requirements**: Need to maintain balance during navigation
- **Step Constraints**: Limited step height and length capabilities
- **Dynamic Stability**: Must maintain dynamic balance during motion
- **Foot Placement**: Precise foot placement required for stable walking

## Path Planning and Obstacle Avoidance

### Global Path Planning for Humanoids

Global path planning in Nav2 for humanoid robots requires special consideration of the robot's physical constraints:

#### Costmap Configuration

Humanoid robots need specialized costmap configurations that account for their unique characteristics:

```yaml
# Example Nav2 costmap configuration for humanoid robot
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      # Humanoid-specific parameters
      resolution: 0.05  # Higher resolution for precise foot placement
      footprint: [[-0.15, -0.1], [-0.15, 0.1], [0.15, 0.1], [0.15, -0.1]]  # Approximate humanoid footprint
      footprint_padding: 0.05  # Reduced padding for narrow spaces

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map
        transform_tolerance: 0.2
        max_publish_frequency: 1.0

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0  # Humanoid height consideration
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # More conservative inflation for humanoid
        inflation_radius: 0.30  # Adjusted for humanoid step size
```

#### Global Planner Selection

For humanoid robots, specific global planners are more appropriate:

- **Navfn**: Good for general pathfinding but needs humanoid-specific modifications
- **Global Planner**: Can be configured for humanoid constraints
- **THETA* or A* variants**: For more precise pathfinding in complex environments

### Local Path Planning and Obstacle Avoidance

Local planning for humanoid robots must consider the robot's dynamic stability and step constraints:

```yaml
# Example local costmap for humanoid navigation
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.05  # Higher resolution for precise planning

      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.30
```

### Humanoid-Specific Navigation Parameters

```yaml
# Example controller server configuration for humanoid
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0  # Higher frequency for humanoid stability
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.1
    min_theta_velocity_threshold: 0.1
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIC"
      debug_trajectory_publish_rate: 10.0
      # Humanoid-specific parameters for step constraints
      lookahead:
        dist: 0.6  # Look ahead distance appropriate for step length
        time: 1.0
      transform_tolerance: 0.1
      shortening_radius: 0.3
      velocity_scaling_tolerance: 0.5
      singularity_angle: 0.174533  # 10 degrees, appropriate for humanoid gait
```

### Obstacle Avoidance Strategies

Humanoid robots require specialized obstacle avoidance strategies:

#### Step-Aware Path Planning
- **Step Height Constraints**: Plan paths that account for maximum step height
- **Step Length Constraints**: Ensure path segments are within step length limits
- **Support Polygon**: Maintain feet within stable support polygon during navigation

#### Dynamic Obstacle Handling
- **Human-Aware Navigation**: Special handling for moving humans in environment
- **Social Navigation**: Consider social norms and personal space
- **Predictive Avoidance**: Predict human movement patterns for smoother navigation

## Navigation Stacks for Bipedal Humanoids

### Humanoid Navigation Architecture

The navigation stack for bipedal humanoids requires integration with the robot's walking controller:

```
Nav2 Navigation Server
         ↓
Path Planning (Global & Local)
         ↓
Trajectory Generation
         ↓
Walking Controller Interface
         ↓
Bipedal Walking Controller
         ↓
Robot Hardware
```

### Walking Controller Integration

Humanoid navigation must be tightly integrated with the walking controller:

```cpp
// Example C++ code for humanoid navigation controller
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"

class HumanoidNavigationController : public rclcpp::Node
{
public:
    HumanoidNavigationController()
    : Node("humanoid_navigation_controller")
    {
        // Subscribe to navigation path
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "local_plan", 10,
            std::bind(&HumanoidNavigationController::pathCallback, this, std::placeholders::_1)
        );

        // Publisher for walking commands
        walking_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "walking_controller/cmd_vel", 10
        );

        // Publisher for footstep plan
        footstep_pub_ = this->create_publisher<footstep_planner_msgs::msg::FootstepArray>(
            "footstep_planner/plan", 10
        );

        // Initialize humanoid-specific parameters
        this->declare_parameter("step_height_max", 0.15);
        this->declare_parameter("step_length_max", 0.4);
        this->declare_parameter("step_time", 0.8);
        this->declare_parameter("balance_margin", 0.05);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        // Convert navigation path to humanoid-appropriate footstep plan
        auto footstep_plan = generateFootsteps(*path_msg);

        // Publish footstep plan to walking controller
        footstep_pub_->publish(footstep_plan);

        // Generate walking commands based on path
        auto walking_cmd = generateWalkingCommands(*path_msg);

        // Publish walking commands
        walking_cmd_pub_->publish(walking_cmd);
    }

    footstep_planner_msgs::msg::FootstepArray generateFootsteps(const nav_msgs::msg::Path& path)
    {
        // Implement humanoid-specific footstep generation
        // considering balance constraints and step limitations
        footstep_planner_msgs::msg::FootstepArray footsteps;

        // Algorithm to convert path to safe footstep sequence
        // considering robot's kinematic constraints

        return footsteps;
    }

    geometry_msgs::msg::Twist generateWalkingCommands(const nav_msgs::msg::Path& path)
    {
        // Generate walking commands that maintain balance
        // while following the navigation path
        geometry_msgs::msg::Twist cmd_vel;

        // Calculate appropriate walking velocity and direction
        // based on path and robot's walking capabilities

        return cmd_vel;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr walking_cmd_pub_;
    rclcpp::Publisher<footstep_planner_msgs::msg::FootstepArray>::SharedPtr footstep_pub_;
};
```

### Footstep Planning Integration

Footstep planning is crucial for humanoid navigation:

#### Step Planning Algorithms
- **A* on Grid**: Traditional path planning adapted for footstep positions
- **RRT-based**: Rapidly-exploring random trees for complex terrain
- **Model Predictive Control**: Optimizes steps based on future predictions

#### Balance Considerations
- **Zero Moment Point (ZMP)**: Maintain ZMP within support polygon
- **Capture Point**: Plan steps to maintain dynamic balance
- **Preview Control**: Use future path information for balance planning

### Humanoid-Specific Navigation Behaviors

#### Stair Navigation
- **Step Detection**: Identify stairs using perception data
- **Step Planning**: Plan precise foot placement on steps
- **Gait Adaptation**: Adjust walking pattern for stair climbing

#### Doorway Navigation
- **Width Assessment**: Determine if doorway is passable
- **Approach Planning**: Plan approach to narrow passages
- **Social Navigation**: Wait for humans to pass through doorways

## Integration with ROS 2 Control Layers

### Control Architecture Overview

The integration between Nav2 and humanoid control layers follows this architecture:

```
High-Level Navigation (Nav2)
         ↓
Mid-Level Motion Planning
         ↓
Low-Level Control (ros2_control)
         ↓
Hardware Interface
```

### ros2_control Integration

The ros2_control framework provides the low-level control interface:

```yaml
# Example ros2_control configuration for humanoid
controller_manager:
  ros__parameters:
    update_rate: 100  # 100Hz update rate for humanoid stability

    # Joint state broadcaster
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # Humanoid-specific controllers
    position_controllers:
      type: position_controllers/JointGroupPositionController
      joints:
        - left_hip_yaw
        - left_hip_roll
        - left_hip_pitch
        - left_knee
        - left_ankle_pitch
        - left_ankle_roll
        - right_hip_yaw
        - right_hip_roll
        - right_hip_pitch
        - right_knee
        - right_ankle_pitch
        - right_ankle_roll
        # ... other joints

    walking_controller:
      type: humanoid_walking_controller/HumanoidWalkingController
      joints:
        - left_hip_yaw
        - left_hip_roll
        - left_hip_pitch
        - left_knee
        - left_ankle_pitch
        - left_ankle_roll
        - right_hip_yaw
        - right_hip_roll
        - right_hip_pitch
        - right_knee
        - right_ankle_pitch
        - right_ankle_roll
```

### Trajectory Execution

Trajectory execution for humanoid robots requires careful coordination:

```python
# Example Python code for trajectory execution
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from humanoid_msgs.msg import WalkingGoal, WalkingStatus
import math

class HumanoidTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('humanoid_trajectory_executor')

        # Trajectory publisher for ros2_control
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/position_controllers/joint_trajectory',
            10
        )

        # Walking controller interface
        self.walking_goal_pub = self.create_publisher(
            WalkingGoal,
            '/walking_controller/goal',
            10
        )

        # Feedback subscribers
        self.joint_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/position_controllers/state',
            self.joint_state_callback,
            10
        )

        self.walking_status_sub = self.create_subscription(
            WalkingStatus,
            '/walking_controller/status',
            self.walking_status_callback,
            10
        )

        # Navigation integration
        self.nav_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.navigation_goal_callback,
            10
        )

    def navigation_goal_callback(self, goal_pose):
        # Convert navigation goal to walking trajectory
        walking_goal = WalkingGoal()
        walking_goal.target_pose = goal_pose.pose
        walking_goal.step_height = 0.05  # Default step height
        walking_goal.step_length = 0.3   # Default step length

        # Publish to walking controller
        self.walking_goal_pub.publish(walking_goal)

    def execute_trajectory(self, trajectory_points):
        # Create joint trajectory message
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll'
            # ... add other joint names
        ]

        # Set trajectory points with appropriate timing
        for point in trajectory_points:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = point.joint_positions
            trajectory_point.velocities = point.joint_velocities
            trajectory_point.accelerations = point.joint_accelerations

            # Calculate time from start based on humanoid walking dynamics
            trajectory_point.time_from_start = Duration(
                sec=0,
                nanosec=int(point.time * 1e9)
            )

            joint_trajectory.points.append(trajectory_point)

        # Publish trajectory
        self.trajectory_pub.publish(joint_trajectory)

def main(args=None):
    rclpy.init(args=args)
    executor = HumanoidTrajectoryExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()
```

### Safety and Emergency Handling

Humanoid navigation systems must include robust safety mechanisms:

#### Balance Recovery
- **Fall Detection**: Detect when robot is losing balance
- **Recovery Actions**: Execute balance recovery behaviors
- **Safe Stop**: Execute safe stopping procedures

#### Collision Avoidance
- **Proximity Detection**: Detect potential collisions
- **Emergency Stop**: Stop immediately when collision is imminent
- **Recovery Planning**: Plan new paths when obstacles appear suddenly

#### System Monitoring
- **Joint Limits**: Monitor for joint limit violations
- **Motor Status**: Monitor motor temperatures and currents
- **Battery Level**: Plan navigation based on available power

## Best Practices for Humanoid Navigation

### Configuration Guidelines

#### Parameter Tuning
- **Costmap Resolution**: Use higher resolution for precise foot placement
- **Inflation Radius**: Adjust for humanoid step size and balance margin
- **Controller Frequency**: Higher frequencies for better balance control
- **Lookahead Distance**: Appropriate for humanoid step length and reaction time

#### Testing Strategies
- **Simulation First**: Test navigation behaviors in simulation
- **Progressive Complexity**: Start with simple environments and increase complexity
- **Edge Cases**: Test with narrow passages, stairs, and dynamic obstacles
- **Long-term Testing**: Validate system stability over extended periods

### Performance Optimization

#### Computational Efficiency
- **Multi-threading**: Use separate threads for perception, planning, and control
- **Efficient Algorithms**: Optimize algorithms for real-time performance
- **Hardware Acceleration**: Use GPU acceleration where possible
- **Memory Management**: Efficient memory usage for continuous operation

#### Energy Efficiency
- **Optimal Gaits**: Use energy-efficient walking patterns
- **Path Optimization**: Plan energy-efficient paths
- **Standby Modes**: Implement low-power modes when stationary
- **Battery Awareness**: Plan navigation considering battery level

### Safety Considerations

#### Human Safety
- **Social Navigation**: Respect human personal space
- **Predictable Behavior**: Ensure navigation behavior is predictable
- **Emergency Stop**: Implement reliable emergency stop mechanisms
- **Collision Avoidance**: Robust collision avoidance systems

#### Robot Safety
- **Joint Limits**: Always respect mechanical joint limits
- **Balance Constraints**: Maintain dynamic balance at all times
- **Environmental Limits**: Avoid environments beyond robot capabilities
- **Graceful Degradation**: Handle sensor failures gracefully

## Summary

Nav2 provides a powerful navigation framework that can be adapted for humanoid robots with proper configuration and integration. The key to successful humanoid navigation lies in understanding the unique challenges of bipedal locomotion and configuring the navigation stack accordingly. Integration with ROS 2 control layers ensures smooth and safe navigation while maintaining the robot's balance and stability.

## Exercises and Thought Experiments

1. **Parameter Tuning**: Consider a humanoid robot that is 1.6m tall with a maximum step height of 0.15m. How would you configure the Nav2 costmaps and planners differently compared to a wheeled robot?

2. **Stair Navigation**: Design a navigation behavior for a humanoid robot to navigate up and down stairs. What special considerations would you need to account for?

3. **Social Navigation**: How would you modify the navigation system to respect social norms when navigating through crowds of people?

## Exercises and Thought Experiments

1. **Parameter Tuning**: Consider a humanoid robot that is 1.6m tall with a maximum step height of 0.15m. How would you configure the Nav2 costmaps and planners differently compared to a wheeled robot?

2. **Stair Navigation**: Design a navigation behavior for a humanoid robot to navigate up and down stairs. What special considerations would you need to account for?

3. **Social Navigation**: How would you modify the navigation system to respect social norms when navigating through crowds of people?

## Next Steps

- Review [Isaac ROS and Hardware-Accelerated Perception](./isaac-ros) to understand perception integration
- Explore [NVIDIA Isaac Sim for Physical AI](./isaac-sim) for simulation and testing