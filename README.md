# bt_nav2_ergocub

Custom Behavior Tree plugins for Nav2, developed for the ergoCub robot navigation stack. The package provides three BT nodes — two condition nodes and one decorator — that extend the standard Nav2 BT library with robot-specific logic.

## Package structure

```
bt_nav2_ergocub/
├── include/bt_nav2_ergocub/
│   ├── decorator_on_bool.hpp
│   ├── is_goal_reached.hpp
│   └── is_path_valid.hpp
├── src/
│   ├── decorator_on_bool.cpp
│   ├── is_goal_reached.cpp
│   └── is_path_valid.cpp
└── bt_descriptions/
    └── replan_on_bool.xml
```

## BT Nodes

### GoalReachedConditionModded (`is_goal_reached`)

**Type:** Condition node

Returns `SUCCESS` when the robot is within the configured position (and optionally angular) tolerance of the goal pose. Returns `FAILURE` otherwise. Also publishes the goal-reached state on `/is_goal_reached`.

This node is designed to sit at the top of a `ReactiveFallback` so that it is re-evaluated every BT tick and can short-circuit ongoing navigation as soon as the robot reaches the goal.

#### BT ports

| Port | Direction | Type | Default | Description |
|------|-----------|------|---------|-------------|
| `goal` | Input | `geometry_msgs::msg::PoseStamped` | — | Target goal pose |
| `global_frame` | Input | `string` | `map` | Global reference frame |
| `robot_base_frame` | Input | `string` | `base_link` | Robot base frame |

#### ROS 2 parameters (set under `bt_navigator`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal_reached_tol` | `double` | `0.3` | XY distance tolerance in metres. Minimum enforced value: `0.05` m |
| `goal_angular_tol` | `double` | `0.14` | Angular tolerance in radians. Minimum enforced value: `0.1` rad |
| `check_angular_alignment` | `bool` | `false` | When `true`, goal is only reached when both XY and yaw tolerances are satisfied |
| `transform_tolerance` | `double` | — | TF lookup time tolerance in seconds |

#### Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `/is_goal_reached` | `std_msgs/Bool` | `true` when goal is reached, `false` otherwise |

#### XML usage

```xml
<GoalReachedConditionModded goal="{goal}"
                            global_frame="map"
                            robot_base_frame="geometric_unicycle"/>
```

---

### IsPathValidConditionModded (`is_path_valid`)

**Type:** Condition node

Calls the Nav2 `is_path_valid` service synchronously. Returns `SUCCESS` if the current path is collision-free, `FAILURE` otherwise. Also publishes the validity state on `/is_path_valid`.

#### BT ports

| Port | Direction | Type | Default | Description |
|------|-----------|------|---------|-------------|
| `path` | Input | `nav_msgs::msg::Path` | — | Path to validate |
| `server_timeout` | Input | `std::chrono::milliseconds` | — | Service call timeout |

#### Called services

| Service | Type | Description |
|---------|------|-------------|
| `is_path_valid` | `nav2_msgs/srv/IsPathValid` | Checks whether the given path is free of collisions |

#### Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `/is_path_valid` | `std_msgs/Bool` | `true` when path is valid, `false` otherwise |

#### XML usage

```xml
<IsPathValidConditionModded path="{path}" server_timeout="2000"/>
```

---

### DecoratorOnBool (`decorator_on_bool`)

**Type:** Decorator node

Gates the execution of its child node based on the response of a ROS 2 `std_srvs/Trigger` service. The child is ticked only when the service returns `success = true` or when the child is already in `RUNNING` state. When the service returns `false` and the child is idle, the decorator returns `SUCCESS` without ticking the child (i.e. it skips replanning).

The primary use case is to gate `ComputePathToPose` so that global path replanning only occurs while the robot is in a double-support phase.

#### BT ports

| Port | Direction | Type | Default | Description |
|------|-----------|------|---------|-------------|
| `service_name` | Input | `string` | `is_on_double_support_srv` | Name of the `std_srvs/Trigger` service to query |

#### Called services

| Service | Type | Description |
|---------|------|-------------|
| `is_on_double_support_srv` (default) | `std_srvs/srv/Trigger` | Returns `success=true` when the robot is in double support, allowing replanning |

#### XML usage

```xml
<DecoratorOnBool service_name="is_on_double_support_srv">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
</DecoratorOnBool>
```

---

## bt_navigator plugin registration

Register all three plugins in the `bt_navigator` section of your Nav2 parameter file:

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - is_path_valid
      - decorator_on_bool
      - is_goal_reached
```

## Example BT: replan on bool

`bt_descriptions/replan_on_bool.xml` shows a minimal tree where global path replanning is gated by `DecoratorOnBool`:

```xml
<PipelineSequence name="NavigateWithReplanning">
  <DecoratorOnBool service_name="is_on_double_support_srv">
    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
  </DecoratorOnBool>
  <FollowPath path="{path}" controller_id="FollowPath"/>
</PipelineSequence>
```

## Dependencies

| Package | Role |
|---------|------|
| `behaviortree_cpp` | BT execution framework |
| `rclcpp` | ROS 2 C++ client library |
| `nav2_util` | Parameter helpers, TF utilities |
| `nav2_msgs` | `IsPathValid` service definition |
| `nav_msgs` | `Path` message |
| `geometry_msgs` | `PoseStamped` message |
| `std_msgs` | `Bool` message |
| `std_srvs` | `Trigger` service definition |
| `tf2_ros` / `tf2_geometry_msgs` | TF2 frame transforms |

## Build

```bash
cd ~/ros2_workspace
colcon build --packages-select bt_nav2_ergocub
source install/setup.bash
```
