# sas_robot_driver

> [!TIP]
> Repository for this module: https://github.com/SmartArmStack/sas_robot_driver. <br/>
> More information: https://smartarmstack.github.io/.

## Quick check

```bash
docker run --rm murilomarinho/sas:jazzy /bin/bash -c "ros2 launch sas_robot_driver sas_robot_driver_ros_composer_example.py"
```

## Contents

- `include/` — public C++ headers.
- `src/` — library and node implementations.
- `scripts/` — Python example scripts.
- `launch/` — example launch files.

## Nodes

- `sas_robot_driver_ros_composer_node` — Composes multiple RobotDriver clients. 
- `sas_robot_watchdog_commander_node` — Sends periodic watchdog triggers to clients.
- `sas_robot_driver_ros_example` — Minimal example robot driver node.

## Launch file

An example to compose multiple `RobotDriver` clients serially.

```bash
ros2 launch sas_robot_driver sas_robot_driver_ros_composer_example.py
```

## Script

Demonstrates `RobotDriverServer` and `RobotDriverClient` in Python.

```bash
ros2 run sas_robot_driver sas_robot_driver_interface_example.py
```

