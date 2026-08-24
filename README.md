# sas_robot_driver

> [!TIP]
> Repository for this module: https://github.com/SmartArmStack/sas_robot_driver. <br/>
> More information: https://smartarmstack.github.io/.

## Quick check

```bash
docker run --rm murilomarinho/sas:jazzy /bin/bash -c "ros2 launch sas_robot_driver composer_example_launch.py"
```

## Contents

- `include/` — public C++ headers.
- `src/` — library and node implementations.
- `scripts/` — Python example scripts.
- `launch/` — example launch files.
- `config/` — example parameter configuration file.

## Client–Server pair

The package implements a **RobotDriverServer / RobotDriverClient** pair.
Both classes take a shared `rclcpp::Node` and a `topic_prefix` string that
forms the namespace for all topics.  When `topic_prefix` is `"GET_FROM_NODE"`
(the default) the node name is used automatically.

### Header files

| Class                    | Header                                         |
|--------------------------|------------------------------------------------|
| `sas::RobotDriverServer` | `sas_robot_driver/sas_robot_driver_server.hpp` |
| `sas::RobotDriverClient` | `sas_robot_driver/sas_robot_driver_client.hpp` |

### Importing in Python

```python
from sas_robot_driver import (
    RobotDriverServer,
    RobotDriverClient,
    Functionality
)
```

## ROS 2 Nodes & Parameters

Each node loads its parameters from a YAML configuration file. The default is
`config/config.yaml` in this package; pass a different file with the
`config_file:=` launch argument of the corresponding launch file.

### Node: `sas_robot_driver_ros_composer_node`

| Property | Value |
|---|---|
| **Executable** | `sas_robot_driver_ros_composer_node` |
| **ROS node name** | `robot_composed` (set by the `name` launch argument of `composer_launch.py`) |
| **Description** | Composes multiple `RobotDriver` clients serially into a single robot driver, exposing one `RobotDriverROS` control loop for all of them. |

#### Parameters

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `robot_driver_client_names` | array of strings | **Mandatory** | none — must be provided | Topic prefixes of the robot-driver clients to compose |
| `override_joint_limits_with_robot_parameter_file` | bool | **Mandatory** | none — must be provided | Whether to take the joint limits from a robot parameter file instead of concatenating the clients' limits |
| `robot_parameter_file_path` | string | **Mandatory** if `override_joint_limits_with_robot_parameter_file` is `true` | none — must be provided | Path of the robot parameter file with the joint limits |
| `thread_sampling_time_sec` | double | **Mandatory** | none — must be provided | Sampling period of the robot control-loop thread |

### Node: `sas_robot_watchdog_commander_node`

| Property | Value |
|---|---|
| **Executable** | `sas_robot_watchdog_commander_node` |
| **ROS node name** | `sas_robot_watchdog_commander_node` |
| **Description** | Sends periodic watchdog triggers to a robot-driver client (joint control is blacklisted). |

#### Parameters

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `thread_sampling_time_sec` | double | **Mandatory** | none — must be provided | Sampling period of the watchdog loop |
| `watchdog_period` | double | **Mandatory** | none — must be provided | Watchdog trigger period |
| `watchdog_maximum_acceptable_delay` | double | **Mandatory** | none — must be provided | Maximum acceptable delay before the watchdog is triggered |
| `robot_name` | string | **Mandatory** | none — must be provided | Topic prefix of the robot-driver client |

### Node: `sas_robot_driver_ros_example`

| Property | Value |
|---|---|
| **Executable** | `sas_robot_driver_ros_example` |
| **ROS node name** | `sas_robot_driver_ros_example` (the launch example runs two instances named `robot_1` and `robot_2`) |
| **Description** | Minimal example robot driver node. |

#### Parameters

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `robot_name` | string | **Mandatory** | none — must be provided | Name of the robot |
| `initial_joint_positions` | array of doubles (radians) | **Mandatory** | none — must be provided | Initial joint positions |
| `joint_limits_min` | array of doubles (radians) | **Mandatory** | none — must be provided | Minimum joint limits |
| `joint_limits_max` | array of doubles (radians) | **Mandatory** | none — must be provided | Maximum joint limits |
| `thread_sampling_time_sec` | double | **Mandatory** | none — must be provided | Sampling period of the robot control-loop thread |

**How mandatory/optional is determined in code:**
- **Mandatory** params are read with `sas::get_ros_parameter(...)` — if missing, the node throws and fails to start.
- **Optional** params are read with `sas::get_ros_optional_parameter(..., <default>)` — they carry in-code defaults.

#### Sample launches

```bash
# Compose two example robots plus a composer node
ros2 launch sas_robot_driver composer_example_launch.py
# Single composer node
ros2 launch sas_robot_driver composer_launch.py
# Watchdog commander
ros2 launch sas_robot_driver watchdog_launch.py
```

## Script

Demonstrates `RobotDriverServer` and `RobotDriverClient` in Python.

```bash
ros2 run sas_robot_driver sas_robot_driver_interface_example.py
```

---

