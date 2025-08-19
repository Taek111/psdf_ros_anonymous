# Repository Guidelines

## Project Structure & Module Organization
- Source (C++): `src/` with headers in `include/psdf_ros/` (PSDF local planner plugin).
- Python nodes and utilities: `scripts/` (e.g., `psdf_ros_node.py`, `mpc_optimizer.py`).
- ROS interfaces: `msg/` (Edge messages), `srv/` (`PsdfMpc.srv`).
- Runtime configs and launches: `config/`, `launch/`, RViz configs in `rviz/`.
- Simulation assets: `maps/`, `worlds/`, optional `urdf/`.
- Tests: `test/` (rostest launch + Python unittest).

## Build, Test, and Development Commands
- Build package (from `~/catkin_ws`): `catkin_make` then `source devel/setup.bash`.
- Build messages/services only after edits: `catkin_make` (CMake regenerates automatically).
- Run node (example): `roslaunch psdf_ros psdf_mpc.launch` or `roslaunch psdf_ros test_psdf_ros.launch`.
- Run tests: `rostest psdf_ros test/test_psdf_node.launch` or `catkin_make run_tests`.

## Coding Style & Naming Conventions
- C++: C++11, two-space indent, braces on same line; classes `UpperCamelCase` (e.g., `PSDFLocalPlanner`); private members end with `_` (e.g., `initialized_`). Keep headers in `include/psdf_ros/`, sources in `src/` and declare in `CMakeLists.txt`.
- Python: four-space indent, modules and functions `snake_case`, classes `CamelCase`. ROS nodes live in `scripts/` and must be executable (`chmod +x`) for `rosrun/roslaunch`.
- Messages/services: keep concise names; update `CMakeLists.txt` `add_*_files` and `generate_messages` when adding.

## Testing Guidelines
- Frameworks: `unittest` + `rostest`.
- Conventions: test files in `test/`, e.g., `test_psdf_service.py`; launch-based tests in `test/*.launch`.
- Run locally: `rostest psdf_ros test/test_psdf_node.launch`. Aim to keep service calls under ~1s as in current tests.

## Commit & Pull Request Guidelines
- Commits: descriptive, imperative subject (e.g., "Add PSDF service timeout"), small scoped changes.
- PRs: include summary, rationale, testing steps (commands used), and affected topics/services. Attach RViz screenshots if UI/visualization changes.
- Keep `CMakeLists.txt`, `package.xml`, and `psdf_local_planner_plugin.xml` in sync when adding plugins, msgs, or services.

## Security & Configuration Tips
- Parameters load from `config/*.yaml` and `launch/*.launch`. Validate topic names and frames (`map`, `odom`, `base_link`).
- The service node uses NumPy/PyTorch; ensure dependencies are installed in your ROS environment if you touch `scripts/` solver code.
