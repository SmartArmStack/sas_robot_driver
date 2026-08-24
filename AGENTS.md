# AGENTS.md

Guidance for AI agents and contributors working in this repository.

## What this is

`sas_robot_driver` is a ROS 2 (jazzy) `ament_cmake` package from the
[SmartArmStack](https://smartarmstack.github.io/) project. It provides a
ROS-topic-based interface to robot drivers: a `RobotDriverServer` (a real robot
driver exposes state and consumes commands) and a `RobotDriverClient` (a
controller sends commands and reads state), both bound to a shared
`rclcpp::Node` and a `topic_prefix`. It is exposed to C++ and to Python
(pybind11).

## Repository layout

- `include/sas_robot_driver/` — public C++ headers
  - `sas_robot_driver_client.hpp` — `sas::RobotDriverClient`
  - `sas_robot_driver_server.hpp` — `sas::RobotDriverServer`
  - `sas_robot_driver_ros.hpp` — `sas::RobotDriverROS` (control loop + watchdog)
- `src/` — library (`sas_robot_driver_server.cpp`, `sas_robot_driver_client.cpp`,
  `sas_robot_driver_ros.cpp`), the pybind11 module (`sas_robot_driver_py.cpp`),
  the composer (`sas_robot_driver_ros_composer.{hpp,cpp}`) and node entry points
- `src/examples/` — minimal example node (`sas_robot_driver_ros_example.cpp`)
- `sas_robot_driver/` — pure-Python package (`__init__.py` re-exports the
  compiled `_sas_robot_driver` pybind11 module)
- `scripts/` — executable Python example scripts (installed with execute perms)
- `launch/` — example launch files
- `pybind11/` — git submodule (pybind11, branch `v3.0`)
- `docker/` — Dockerfile + compose used by CI for the build/test run

## Build, run, verify

There is no local test suite; verification is "it builds and the example
script runs". All builds happen inside the prebuilt SmartArmStack image
`murilomarinho/sas:jazzy`, which provides `ros2`, `colcon`, `Eigen3`,
`-ldqrobotics`, and the sibling SAS packages.

Quick check (what CI runs):

```bash
docker compose build -f docker/compose.yml
docker compose up -f docker/compose.yml
# or the one-shot version from the README:
docker run --rm murilomarinho/sas:jazzy /bin/bash -c "ros2 launch sas_robot_driver sas_robot_driver_ros_composer_example.py"
```

Building outside docker (requires a ROS 2 jazzy workspace with the sibling
packages `sas_common`, `sas_core`, `sas_conversions`, `sas_msgs`, plus
`Eigen3` and `libdqrobotics` on the system):

```bash
# from the parent of this folder (e.g. /root/sas_robot_driver_devel/src/):
colcon build
source install/setup.bash
```

Examples:

```bash
ros2 launch sas_robot_driver sas_robot_driver_ros_composer_example.py
ros2 run sas_robot_driver sas_robot_driver_interface_example.py
```

CI (`.github/workflows/build.yml`) does `docker compose build` then
`docker compose up`, which builds with `colcon` and runs the Python
interface example. Make sure changes keep that flow green.

## Architecture notes

- Client/server pairing: `RobotDriverClient` and `RobotDriverServer` take
  `(shared_ptr<rclcpp::Node>, topic_prefix)`. With `topic_prefix ==
  "GET_FROM_NODE"` (the default) the node name is used. Topics are
  namespaced under the prefix, so a client and server only connect when
  their prefixes match.
- Client enablement: the client is only enabled after it receives joint
  states and joint limits from its server (`is_enabled()`); servers that
  wait on a client should send `send_joint_states()` / `send_joint_limits()`
  while spinning.
- Mode blacklisting: `RobotDriverClient` accepts `blacklisted_modes`
  (`MODE_BLACKLIST_FLAG`, e.g. `JOINT_CONTROL`) to disable functionality
  from the client side — the watchdog commander node uses this.
- `RobotDriverROS` wraps any `sas::RobotDriver` with the ROS control loop
  (`control_loop()`) and an optional watchdog
  (`RobotDriverROSConfiguration::watchdog_period_in_seconds`; <= 0 disables).
- `RobotDriverROSComposer` composes multiple clients into one driver
  (concatenating joint limits, or overriding them from a robot parameter
  file). CoppeliaSim support was moved out to `sas_robot_driver_coppeliasim`
  in 2025 — do not re-add it here.
- Python API (compiled via `src/sas_robot_driver_py.cpp`, module
  `_sas_robot_driver`): `RobotDriverServer`, `RobotDriverClient`,
  `Functionality`, `MODE_BLACKLIST_FLAG`, `RobotDriverROS`,
  `RobotDriverROSConfiguration`. Python uses `sas_common` helpers
  (`rclcpp_init`, `rclcpp_Node`, `rclcpp_spin_some`, `rclcpp_shutdown`).

## ROS parameters

Nodes read configuration via `sas::get_ros_parameter(node, name, value)`
(typically in launch files or `ros2 run` parameters):

- `sas_robot_driver_ros_example`: `robot_name`, `initial_joint_positions`,
  `joint_limits_min`, `joint_limits_max`, `thread_sampling_time_sec`
- `sas_robot_watchdog_commander_node`: `robot_name`, `thread_sampling_time_sec`,
  `watchdog_period`, `watchdog_maximum_acceptable_delay`
- `sas_robot_driver_ros_composer_node`: `robot_driver_client_names`,
  `override_joint_limits_with_robot_parameter_file`,
  `robot_parameter_file_path` (only if override is true),
  `thread_sampling_time_sec`

## Conventions

- License: LGPLv3. Every source/header/script file starts with the standard
  copyright/license/comment block used throughout this repo — keep it when
  creating or touching files, and keep the "Contributors:" section current
  for non-trivial additions.
- C++ compiles with `-Wall -Wextra -Wpedantic`; keep new code warning-clean.
- Be aware the client/server .cpp files are compiled twice (once for the
  shared library, once into the pybind11 module) — keep Python-specific
  code in `src/sas_robot_driver_py.cpp`.
- External dependency: the code links against `dqrobotics`
  (`-ldqrobotics`, plus `dqrobotics-interface-json11` for the composer
  node) — a system library shipped with the SAS docker image, not a ROS
  package.
- When changing `CMakeLists.txt`, note the pybind11 import block marked
  `pybind11 import block [BEGIN]/[END]` is copied across the SAS repos —
  keep its delimiters intact.
- Python is linted with `ament_flake8` / `ament_pep257` (declared in
  `package.xml` as test deps); no `.flake8` override file is present.
- No unit tests exist in this package; don't add test frameworks unless
  asked.

## Git

- Default/development branch is `jazzy` (ROS 2 distribution branch naming).
- Shallow clones may lack full history; `git fetch --unshallow` if needed.
- `pybind11` is a submodule — initialize with
  `git submodule update --init --recursive` after cloning.
