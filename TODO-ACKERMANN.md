# TODO – Ackermann Support for PSDF-MPC

## Modeling & Parameters
- [x] Decide on the Ackermann dynamic model (state `[x, y, yaw, steering]`, control `[speed, steering_rate]`) and document required vehicle constants such as wheelbase and steering limits.
- [x] Extend runtime parameters (ROS params + YAML) to expose `vehicle_model`, `wheelbase`, steering angle bounds, and rate limits alongside existing differential drive bounds.

## `scripts/utils.py`
- [x] Generalize `State` usage to support Ackermann state/control by introducing an `AckermannSystem` companion to `DifferentialDriveSystem` (`scripts/utils.py`).
- [x] Implement forward dynamics and `nominal_safe_controller` variants aware of steering limits.
- [x] Ensure geometry helpers still work (shared `_geometry`) and expose wheelbase where needed.

## `scripts/mpc_optimizer.py`
- [x] Parameterize `PSDFOptimizer` with the selected vehicle model and branch `create_model` for Ackermann dynamics.
- [x] Update `nx`, `nu`, cost matrices, bounds, and warm-start logic for Ackermann mode.
- [x] Add steering angle and rate constraints to the OCP mirroring the existing `v/ω` bounds.
- [x] Revisit output extraction so the solver returns speed + steering-rate and exposes steering telemetry.

## `scripts/psdf_ros_node.py`
- [x] Parse `vehicle_model`/Ackermann parameters from ROS and instantiate the proper system in `create_system`.
- [x] Update `extract_state` to recover steering from odometry and push `[x, y, yaw, steering]` into the MPC state vector.
- [x] Interpret optimizer output per model and convert Ackermann commands back to Twist (encoding steering data in spare channels).
- [x] Ensure reference generation/debug publishers handle the additional steering state.

## `src/psdf_local_planner.cpp`
- [x] Keep planner/service interface in `Twist` while Ackermann yaw-rate conversion happens inside the service node (no C++ change required).
- [x] Reuse existing odometry subscription; steering state is inferred within `psdf_ros_node.py`, so no additional planner wiring is needed.

## Documentation & Validation
- [x] Refresh `.cursor/rules/project-description.mdc` and user guides with the new Ackermann model description and parameters.
- [ ] Add simulation or rostest scenarios exercising Ackermann kinematics (e.g., curved path, steering saturation) and ensure regression coverage for differential-drive mode.
- [ ] Document migration steps for existing users (parameter examples, service changes) before cutting releases.
