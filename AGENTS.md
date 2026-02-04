# Repository Guidelines

## Project Structure & Module Organization
- `src/psdf_local_planner.cpp` and `include/psdf_ros/` implement the nav_core local planner plugin exposed to `move_base`.
- `src/psdf_ros_node.py` exposes the `/psdf_mpc` service node, while `scripts/` houses other Python nodes, MPC utilities, and obstacle-processing helpers; keep reusable logic modular.
- `config/` contains YAML parameters for planners, robot footprints, and MPC tuning, while `launch/` supplies bring-up scenarios (`test_psdf.launch`, CARLA integrations, etc.).
- Interfaces live in `msg/` and `srv/`; rebuild after editing them to refresh generated code.
- `test/` stores rostest launch files and pytest-style service checks; `references/` holds research prototypes that should guide—but not duplicate—runtime code.

## Build, Test, and Development Commands
- `cd ~/catkin_ws && catkin_make --pkg psdf_ros` builds messages, the C++ plugin, and installs Python entry points.
- `source ~/catkin_ws/devel/setup.bash` before running tools so custom messages and plugins resolve.
- `rostest psdf_ros test/test_psdf_node.launch` executes the service integration test headlessly.
- `roslaunch psdf_ros test_psdf.launch` brings up a lightweight stack for manual verification; `rosrun psdf_ros psdf_ros_node.py` exposes the `/psdf_mpc` service without a launch file.

## Coding Style & Naming Conventions
- C++ follows ROS conventions: 2-space indentation, braces on the same line, `UpperCamelCase` classes, `snake_case` members with trailing underscores (`service_name_`).
- Python adopts PEP 8 with 4-space indentation, explicit docstrings, and type hints where practical; keep modules lower_snake_case (e.g., `psdf_wrapper.py`).
- Use ROS logging macros (`ROS_INFO`, `rospy.loginfo`) and namespace topics/services consistently in lower_snake_case.

## Testing Guidelines
- Prefer rostest-based integration like `test_psdf_service.py`, which wraps `unittest`—mirror that structure for new service or topic contracts.
- When adding algorithms, supply deterministic fixtures or bag extracts under `test/` and assert timing budgets (see existing <1s expectation).
- Document manual validation steps in PRs when tests require hardware or simulation.

## Commit & Pull Request Guidelines
- Keep commits short, present-tense, and focused (e.g., `add carla simulation environment`, `fix corridor_world.world`).
- PRs should summarize behavior changes, list test commands executed, link tracking issues, and include screenshots or logs for visualization updates.
- Highlight interface changes to `msg/`, `srv/`, or launch files so downstream packages can react.
