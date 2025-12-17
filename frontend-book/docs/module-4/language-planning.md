---
id: language-planning
title: Language-Driven Cognitive Planning
sidebar_label: Language-Driven Planning
---

# Chapter 2: Language-Driven Cognitive Planning

## Learning Objectives

By the end of this chapter, students will be able to:
- Utilize Large Language Models (LLMs) for task decomposition in robotic systems
- Translate natural language commands into ROS 2 action sequences
- Implement safety and grounding mechanisms for language-based plans
- Design cognitive architectures that bridge high-level goals with low-level robot actions

## Introduction

Language-driven cognitive planning represents a paradigm shift in robotics, enabling robots to understand and execute complex tasks expressed in natural language. This approach allows humans to communicate high-level goals to robots without requiring detailed knowledge of robotic control systems. The chapter explores how LLMs can decompose complex tasks, generate executable action sequences, and ensure safe execution in real-world environments.

## Using LLMs for Task Decomposition

### Overview of Task Decomposition

Task decomposition is the process of breaking down complex, high-level goals into sequences of simpler, executable actions. In the context of robotics, this involves translating human intentions into specific robot behaviors that can be executed within the constraints of the robotic system and its environment.

### LLM-Based Task Decomposition Approaches

Large Language Models excel at understanding the structure of complex tasks and can be prompted to decompose high-level goals into executable steps. The key is to provide the LLM with sufficient context about the robot's capabilities, environment, and available actions.

### Context-Aware Task Decomposition

```python
import openai
from typing import List, Dict, Any

class TaskDecomposer:
    def __init__(self, robot_capabilities: Dict[str, Any]):
        self.client = openai.OpenAI()
        self.robot_capabilities = robot_capabilities

    def decompose_task(self, natural_language_goal: str) -> List[Dict[str, Any]]:
        prompt = f"""
        Decompose the following high-level goal into executable robotic actions:
        Goal: "{natural_language_goal}"

        Robot capabilities:
        - Manipulation: {self.robot_capabilities.get('manipulation', False)}
        - Navigation: {self.robot_capabilities.get('navigation', False)}
        - Perception: {self.robot_capabilities.get('perception', False)}
        - Max payload: {self.robot_capabilities.get('max_payload_kg', 'N/A')} kg

        Available actions: move, grasp, navigate, inspect, place, pick, detect

        Environment: indoor_lab with obstacles, furniture, and various objects

        Return a JSON array of action objects with the following structure:
        [
          {{
            "action": "action_type",
            "target": {{"name": "object_name", "position": [x, y, z], "properties": "..."}},
            "description": "Human-readable description of the action",
            "preconditions": ["list of preconditions that must be met"],
            "effects": ["list of effects of the action"]
          }}
        ]

        Ensure the task decomposition is complete, executable, and considers safety constraints.
        """

        response = self.client.chat.completions.create(
            model="gpt-4-turbo",
            messages=[{"role": "user", "content": prompt}],
            response_format={"type": "json_object"}
        )

        import json
        result = json.loads(response.choices[0].message.content)

        # Extract the action sequence from the result
        if "action_sequence" in result:
            return result["action_sequence"]
        elif isinstance(result, list):
            return result
        else:
            return []
```

### Example Task Decomposition

Consider the high-level command: "Clean the room by putting the books on the shelf and disposing of the trash"

The LLM might decompose this into:
1. **Perception Phase**: Identify all books and trash items in the room
2. **Navigation Phase**: Move to the first book location
3. **Manipulation Phase**: Grasp the book
4. **Navigation Phase**: Move to the shelf
5. **Manipulation Phase**: Place the book on the shelf
6. **Repeat** for all books
7. **Navigation Phase**: Move to the trash location
8. **Manipulation Phase**: Grasp the trash
9. **Navigation Phase**: Move to the disposal area
10. **Manipulation Phase**: Dispose of the trash

### Hierarchical Task Networks

LLMs can generate hierarchical task networks that organize actions at multiple levels of abstraction:

```python
class HierarchicalTaskNetwork:
    def __init__(self):
        self.tasks = []
        self.subtasks = {}
        self.primitives = []

    def generate_htn(self, goal: str) -> 'HierarchicalTaskNetwork':
        # Generate high-level tasks first
        high_level_tasks = self.decompose_high_level(goal)

        htn = HierarchicalTaskNetwork()
        htn.tasks = high_level_tasks

        # Decompose each high-level task into subtasks
        for task in high_level_tasks:
            subtasks = self.decompose_task(task['description'])
            htn.subtasks[task['id']] = subtasks

        return htn
```

## Translating Natural Language to ROS 2 Action Sequences

### Natural Language Understanding Pipeline

The translation from natural language to ROS 2 action sequences involves several stages:

1. **Semantic Parsing**: Convert natural language into a structured representation
2. **Action Mapping**: Map semantic concepts to specific ROS 2 actions
3. **Parameter Extraction**: Extract parameters needed for action execution
4. **Constraint Validation**: Verify that the plan is executable given robot and environment constraints

### ROS 2 Action Sequence Generation

```python
from rclpy.action import ActionClient
from robot_actions.action import NavigateToPose, ManipulateObject
import json

class LanguageToActionTranslator:
    def __init__(self, node):
        self.node = node
        self.navigate_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.manipulate_client = ActionClient(node, ManipulateObject, 'manipulate_object')

    def translate_and_execute(self, natural_language_command: str):
        # Get the action sequence from the LLM
        action_sequence = self.get_action_sequence(natural_language_command)

        # Execute each action in the sequence
        for action in action_sequence:
            self.execute_action(action)

    def get_action_sequence(self, command: str) -> List[Dict[str, Any]]:
        # Use LLM to generate action sequence
        decomposer = TaskDecomposer(self.get_robot_capabilities())
        return decomposer.decompose_task(command)

    def execute_action(self, action: Dict[str, Any]):
        action_type = action['action']

        if action_type == 'navigate':
            self.execute_navigation(action)
        elif action_type == 'grasp':
            self.execute_manipulation(action)
        elif action_type == 'place':
            self.execute_manipulation(action)
        # Add more action types as needed

    def execute_navigation(self, action: Dict[str, Any]):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_from_target(action['target'])

        self.navigate_client.wait_for_server()
        future = self.navigate_client.send_goal_async(goal_msg)
        # Handle the response asynchronously

    def execute_manipulation(self, action: Dict[str, Any]):
        goal_msg = ManipulateObject.Goal()
        goal_msg.operation = action['action']
        goal_msg.object = self.create_object_from_target(action['target'])

        self.manipulate_client.wait_for_server()
        future = self.manipulate_client.send_goal_async(goal_msg)
        # Handle the response asynchronously
```

### Action Schema Definition

To ensure consistent translation, define action schemas that specify how natural language should be mapped to ROS 2 actions:

```python
ACTION_SCHEMAS = {
    "navigate": {
        "required_params": ["target_position"],
        "optional_params": ["orientation", "speed"],
        "ros_action": "NavigateToPose",
        "description": "Move the robot to a specified position"
    },
    "grasp": {
        "required_params": ["object_id", "grasp_type"],
        "optional_params": ["force", "approach_direction"],
        "ros_action": "ManipulateObject",
        "description": "Grasp an object with the robot's end effector"
    },
    "place": {
        "required_params": ["object_id", "target_position"],
        "optional_params": ["orientation", "release_force"],
        "ros_action": "ManipulateObject",
        "description": "Place an object at a specified location"
    }
}
```

### Safety-First Translation

When translating natural language to actions, safety constraints must be considered:

```python
class SafeLanguageTranslator:
    def __init__(self, node):
        self.node = node
        self.safety_checker = SafetyChecker(node)

    def translate_safely(self, command: str) -> List[Dict[str, Any]]:
        # Get initial action sequence
        action_sequence = self.get_action_sequence(command)

        # Validate each action for safety
        safe_sequence = []
        for action in action_sequence:
            if self.safety_checker.is_safe(action):
                safe_sequence.append(action)
            else:
                # Try to find a safe alternative
                safe_alternative = self.safety_checker.get_safe_alternative(action)
                if safe_alternative:
                    safe_sequence.append(safe_alternative)
                else:
                    raise ValueError(f"No safe alternative for action: {action}")

        return safe_sequence
```

## Safety and Grounding of Language-Based Plans

### Safety Considerations in Language-Driven Planning

Language-driven plans must incorporate multiple layers of safety to prevent harmful or impossible actions:

1. **Physical Safety**: Ensure actions don't cause harm to humans or damage to property
2. **Logical Safety**: Verify that actions are logically possible and consistent
3. **Environmental Safety**: Consider environmental constraints and obstacles
4. **Capability Safety**: Confirm that the robot can physically perform the requested actions

### Safety Validation Framework

```python
class SafetyValidator:
    def __init__(self, robot_state, environment_model):
        self.robot_state = robot_state
        self.environment_model = environment_model

    def validate_plan(self, action_sequence: List[Dict[str, Any]]) -> Dict[str, Any]:
        validation_results = {
            "is_safe": True,
            "violations": [],
            "suggestions": []
        }

        for i, action in enumerate(action_sequence):
            # Check physical safety
            physical_check = self.check_physical_safety(action)
            if not physical_check["safe"]:
                validation_results["is_safe"] = False
                validation_results["violations"].append({
                    "action_index": i,
                    "type": "physical",
                    "message": physical_check["message"]
                })

            # Check logical consistency
            logical_check = self.check_logical_consistency(action, action_sequence[:i])
            if not logical_check["safe"]:
                validation_results["is_safe"] = False
                validation_results["violations"].append({
                    "action_index": i,
                    "type": "logical",
                    "message": logical_check["message"]
                })

            # Check environmental constraints
            env_check = self.check_environmental_constraints(action)
            if not env_check["safe"]:
                validation_results["is_safe"] = False
                validation_results["violations"].append({
                    "action_index": i,
                    "type": "environmental",
                    "message": env_check["message"]
                })

        return validation_results

    def check_physical_safety(self, action: Dict[str, Any]) -> Dict[str, Any]:
        # Check if action violates physical constraints
        if action["action"] == "grasp":
            object_weight = self.get_object_weight(action["target"])
            if object_weight > self.robot_state["max_payload"]:
                return {"safe": False, "message": f"Object too heavy: {object_weight}kg exceeds max payload"}

        return {"safe": True, "message": "Physical safety check passed"}

    def check_logical_consistency(self, action: Dict[str, Any], previous_actions: List[Dict[str, Any]]) -> Dict[str, Any]:
        # Check if action is logically consistent with previous actions
        if action["action"] == "place":
            # Check if object was previously grasped
            object_grasped = any(
                prev_action["action"] == "grasp" and
                prev_action["target"]["name"] == action["target"]["object_name"]
                for prev_action in previous_actions
            )
            if not object_grasped:
                return {"safe": False, "message": f"Cannot place object that wasn't grasped: {action['target']['object_name']}"}

        return {"safe": True, "message": "Logical consistency check passed"}

    def check_environmental_constraints(self, action: Dict[str, Any]) -> Dict[str, Any]:
        # Check if action violates environmental constraints
        if action["action"] == "navigate":
            path = self.plan_path(action["target"]["position"])
            if not path or self.path_has_obstacles(path):
                return {"safe": False, "message": "Navigation path has obstacles or is blocked"}

        return {"safe": True, "message": "Environmental constraints check passed"}
```

### Grounding Language-Based Plans

Grounding ensures that abstract language plans are connected to concrete, perceivable elements in the environment:

```python
class PlanGroundingSystem:
    def __init__(self, perception_system, knowledge_base):
        self.perception_system = perception_system
        self.knowledge_base = knowledge_base

    def ground_plan(self, action_sequence: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        grounded_sequence = []

        for action in action_sequence:
            grounded_action = action.copy()

            # Ground object references
            if "target" in action and "name" in action["target"]:
                grounded_object = self.ground_object(action["target"]["name"])
                if grounded_object:
                    grounded_action["target"]["grounded_object"] = grounded_object
                else:
                    raise ValueError(f"Could not ground object reference: {action['target']['name']}")

            # Ground location references
            if "target" in action and "location" in action["target"]:
                grounded_location = self.ground_location(action["target"]["location"])
                if grounded_location:
                    grounded_action["target"]["grounded_position"] = grounded_location
                else:
                    raise ValueError(f"Could not ground location reference: {action['target']['location']}")

            grounded_sequence.append(grounded_action)

        return grounded_sequence

    def ground_object(self, object_name: str) -> Dict[str, Any]:
        # Find the actual object in the environment
        detected_objects = self.perception_system.detect_objects()

        # Use knowledge base to find object by name
        for obj in detected_objects:
            if self.matches_name(obj, object_name):
                return obj

        # If not found in current perception, try knowledge base
        return self.knowledge_base.get_object_location(object_name)

    def ground_location(self, location_name: str) -> List[float]:
        # Ground location references like "left corner", "shelf", etc.
        if location_name in self.knowledge_base.predefined_locations:
            return self.knowledge_base.predefined_locations[location_name]

        # Try to find location based on spatial relations
        if "corner" in location_name:
            return self.find_corner_location(location_name)

        return None
```

### Runtime Safety Monitoring

Safety must be continuously monitored during plan execution:

```python
class RuntimeSafetyMonitor:
    def __init__(self, node):
        self.node = node
        self.current_plan = None
        self.executed_actions = []

    def monitor_execution(self, plan: List[Dict[str, Any]]):
        self.current_plan = plan
        self.executed_actions = []

        for action in plan:
            # Check safety before executing
            if not self.is_safe_to_execute(action):
                self.node.get_logger().warn(f"Unsafe action detected: {action}")
                self.abort_execution()
                return False

            # Execute action
            result = self.execute_action(action)

            # Update executed actions
            self.executed_actions.append({
                "action": action,
                "result": result,
                "timestamp": self.node.get_clock().now()
            })

            # Check for safety violations after execution
            if not self.check_post_execution_safety(action, result):
                self.node.get_logger().warn(f"Post-execution safety violation: {action}")
                return False

        return True

    def is_safe_to_execute(self, action: Dict[str, Any]) -> bool:
        # Check if action is safe to execute given current state
        current_state = self.get_current_robot_state()

        # Verify robot capabilities
        if not self.verify_capabilities(action, current_state):
            return False

        # Check environment for safety
        if not self.verify_environment_safety(action):
            return False

        return True
```

## Advanced Planning Techniques

### Multi-Modal Planning

Language-driven planning can be enhanced with visual and sensory information:

```python
class MultiModalPlanner:
    def __init__(self, language_model, vision_system, robot_interface):
        self.language_model = language_model
        self.vision_system = vision_system
        self.robot_interface = robot_interface

    def create_plan(self, natural_language_goal: str) -> List[Dict[str, Any]]:
        # Get initial plan from language model
        initial_plan = self.language_model.decompose_task(natural_language_goal)

        # Enhance plan with visual information
        enhanced_plan = []
        for action in initial_plan:
            if self.needs_visual_verification(action):
                # Use vision system to verify/adjust parameters
                visual_info = self.vision_system.get_relevant_info(action)
                enhanced_action = self.update_action_with_visual_info(action, visual_info)
                enhanced_plan.append(enhanced_action)
            else:
                enhanced_plan.append(action)

        return enhanced_plan
```

### Plan Refinement and Adaptation

Real-world execution often requires plan refinement based on feedback:

```python
class AdaptivePlanner:
    def __init__(self):
        self.original_plan = None
        self.current_step = 0

    def adapt_plan(self, current_state: Dict[str, Any], feedback: Dict[str, Any]) -> List[Dict[str, Any]]:
        if not feedback.get("success", True):
            # Plan failed, need to adapt
            return self.generate_alternative_plan(current_state, feedback)
        else:
            # Plan is proceeding normally, continue with remaining steps
            return self.original_plan[self.current_step+1:]

    def generate_alternative_plan(self, current_state: Dict[str, Any], failure_info: Dict[str, Any]) -> List[Dict[str, Any]]:
        # Generate a new plan considering the failure and current state
        # This might involve going back to the LLM with updated context
        pass
```

## Summary

Language-driven cognitive planning enables robots to understand and execute complex tasks expressed in natural language. Key components include:

1. **Task Decomposition**: Using LLMs to break down high-level goals into executable actions
2. **Action Translation**: Converting natural language to ROS 2 action sequences
3. **Safety and Grounding**: Ensuring plans are safe and connected to the real world
4. **Runtime Monitoring**: Continuously verifying safety during execution

These capabilities enable more intuitive human-robot interaction, allowing users to communicate complex tasks in natural language while ensuring safe and reliable execution by robotic systems.

## Exercises

1. Implement a task decomposition system using an LLM for a specific robotic domain
2. Create a safety validation framework for language-driven plans
3. Design a grounding system that connects language references to perceptual entities
4. Build a runtime monitoring system that adapts plans based on execution feedback