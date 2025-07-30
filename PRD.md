# PSDF-ROS: PSDF-MPC Local Planner for ROS1 (Melodic)

## Overview and Motivation

The **PSDF-ROS** package provides a model-predictive local planner for ROS1 (Melodic) that replaces the default ROS `move_base` local planner with a **PSDF-MPC controller**. _PSDF_ stands for **Polygonal Signed Distance Field**, a technique to compute the signed distance between a convex robot footprint and multiple obstacles (each represented as a cluster of line segments) in a fully differentiable, branch-free way. The PSDF-MPC approach uses this differentiable signed distance field within a **Model Predictive Control (MPC)** framework to achieve collision avoidance while smoothly tracking the global path.


**Motivation:** The default planners (like DWA or Trajectory Rollout) use heuristic costmaps and sampling which can struggle with dynamic constraints or complex obstacle shapes. PSDF-MPC provides a more principled approach: it **optimizes a control trajectory** by minimizing a cost (tracking error to the global plan and control effort) subject to the robot's dynamics and a **signed distance constraint** to keep a safe clearance from obstacles. This yields smoother and more optimal motions with formal safety margins. The goal is to run this advanced controller **in real-time (≥ 10 Hz)** on typical robot computers (CPU-only, e.g. NVIDIA Jetson Xavier or Intel i7), ensuring stable performance and safety. By integrating with ROS `move_base`, the PSDF-ROS package allows easy drop-in replacement of the local planner in existing navigation stacks, providing improved obstacle avoidance capabilities and more flexible obstacle representations (using segments instead of grid occupancy).

 

In summary, **PSDF-ROS** aims to improve local navigation by combining:

- _Differentiable collision checking:_ The PSDF computes continuous gradients of distance to obstacles, enabling efficient gradient-based optimization for collision avoidance.
    
- _MPC optimal control:_ The local planner optimizes velocities over a short horizon (~1–2 seconds) to minimize path tracking error and control effort while respecting dynamic limits and obstacle constraints.
    
- _ROS integration:_ Provided as a plugin to `move_base` and a companion ROS node, so it fits into the standard ROS1 navigation pipeline with minimal changes.
    

## System Architecture

The PSDF-ROS system consists of two core components working together within the ROS navigation framework:

- **`psdf_local_planner`** – a C++ local planner plugin for `move_base` (conforming to `nav_core::BaseLocalPlanner`). This plugin replaces the default local planner. It receives the global plan and current robot state from `move_base`, and calls the PSDF-MPC service to compute velocity commands.
    
- **`psdf_ros`** – a Python ROS node acting as a service server. It encapsulates the PSDF-MPC optimizer and an obstacle processing pipeline. The node maintains the latest obstacle information (as clusters of line segments) and uses the PSDF library and MPC solver (ACADOS with CasADi and PyTorch backends) to compute optimal control outputs in response to service calls.
    

The high-level architecture and data flow are illustrated below:

```
           Global Plan (nav_msgs/Path)
                 ↓
        [ move_base ] 
        (global planner + costmaps)
                 |
                 | setPlan(global_plan)
                 v
     [ psdf_local_planner (C++ plugin) ]                Sensor data (LiDAR, etc.)
        |    (implements BaseLocalPlanner)                      ↓
        |--- (Service request: /psdf_mpc) --------------> [ Obstacle Detector ]
        |                                               (extracts edge segments)
        |                                        updates
        |                                        obstacles
        |              <--- Twist cmd (Service response) ---|
        v                                                  |
   /cmd_vel published                                [ psdf_ros Node (Python) ]
   (to robot base)                                    - PSDF Optimizer (MPC)
                                                     - Obstacle Updater (Edge clusters)

```

In this architecture, `move_base` provides the global plan (in the `map` frame) and invokes the `psdf_local_planner` at a fixed frequency (e.g. 10 Hz). The local planner plugin uses the robot’s current pose/velocity and a segment of the global plan to formulate a request to the PSDF-MPC **ROS service** (`/psdf_mpc`). The `psdf_ros` service node, running the PSDF-MPC optimizer, computes an optimal velocity command avoiding obstacles and returns it to the plugin. The plugin then passes this velocity to `move_base`, which publishes it on `/cmd_vel` to drive the robot.

 

Obstacle information is continuously updated in the `psdf_ros` node. A separate **Obstacle Detector** (which can be part of `psdf_ros` or an external module) processes sensor or costmap data to produce **edge segment clusters**. These clusters represent obstacle boundaries (e.g. walls or object outlines) and are fed into the PSDF model. The PSDF logic computes the signed distance to these edges for any given robot pose and ensures the MPC respects a minimum clearance.

 

This design cleanly separates concerns:

- The C++ plugin handles ROS integration, synchronization with `move_base`, and emergency fallback logic (like stopping if the service fails).
    
- The Python `psdf_ros` node handles heavy computations (distance field & MPC solve) and can leverage libraries like PyTorch and CasADi/ACADOS. This separation allows easier debugging and the option to run the optimizer on a different process or machine if needed.
    

## Package Directory Structure

The `psdf_ros` package is organized as follows:
```
psdf_ros/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── psdf_ros/
│       └── psdf_local_planner.h        # C++ interface for the local planner plugin
├── src/
│   ├── psdf_local_planner.cpp         # C++ plugin implementation
├── scripts/
│   ├── psdf_ros_node.py               # Python node providing the /psdf_mpc service
│   ├── psdf_optimizer.py              # PSDF-MPC optimizer logic (wraps PSDF & ACADOS)
│   ├── psdf.py                        # PSDF library (differentiable distance field):contentReference[oaicite:3]{index=3}
│   ├── obstacle_detector.py           # Obstacle processing (segments clustering, etc.)
│   └── ... (other modules such as psdf_wrapper, geometry utils, etc.)
├── msg/
│   ├── EdgeSegment.msg                # Definition of a line segment (obstacle edge)
│   ├── EdgeCluster.msg                # Group of edge segments (one obstacle)
│   └── EdgeClusters.msg               # Array of EdgeCluster (all obstacles)
├── srv/
│   └── PsdfMpc.srv                    # Service definition for PSDF-MPC requests
├── config/
│   ├── psdf_local_planner.yaml        # Parameters for the local planner & optimizer
│   └── robot_footprint.yaml           # Robot geometry/footprint definition (convex polygon)
└── launch/
    └── psdf_mpc.launch.xml            # Launch file for the PSDF-ROS system

```

Some notable aspects:

- The C++ plugin is built as a shared library and exported via pluginlib, so `move_base` can load it using the plugin name `psdf_local_planner/PSDFLocalPlanner`.
    
- The Python node relies on external dependencies (PyTorch, CasADi, ACADOS) which must be installed in the environment. These are declared in `package.xml` (though binary installation might be manual for ACADOS).
    
- The `msg/` definitions allow the system to accept obstacle inputs as line segments and clusters, abstracting away the source of obstacle data. (For example, you could have a node that converts a costmap or LiDAR scan into `EdgeClusters` message.)
    
- Configuration is separated into YAML files for clarity. The `psdf_local_planner.yaml` configures planning parameters (MPC horizon, weights, limits, etc.), and the `robot_footprint.yaml` (or parameters within the main config) defines the robot’s footprint geometry.
    

## Core Components

### PSDF Local Planner Plugin (C++)

The `psdf_local_planner` is a C++ class that implements the ROS `nav_core::BaseLocalPlanner` interface, making it compatible with `move_base`. Its responsibilities include:

- **Receiving the Global Plan:** When `move_base` calls `initialize` and `setPlan`, the plugin stores the global plan (as a sequence of `geometry_msgs::PoseStamped`) that it should follow. It may transform this plan into the local planning frame (e.g., `odom`) for easier processing.
    
- **Periodic Command Computation:** On each cycle (e.g., 10 Hz), `move_base` calls `computeVelocityCommands(Twist& cmd_vel)`. The plugin will:
    
    1. Determine the current robot pose (from the localization system or TF in the `odom` frame) and current velocity (from odometry).
        
    2. Select a local reference trajectory for the next short horizon. Typically, this could be the first segment of the global plan ahead of the robot (e.g., the path points covering ~1–2 seconds of travel) or simply the final goal if it’s within the horizon distance. The reference trajectory is used by the MPC to track the desired path.
        
    3. Populate a service request (`PsdfMpc.srv`) with the current state and the reference trajectory (see **ROS Interfaces** below for details).
        
    4. Call the `/psdf_mpc` service (using a ROS service client). A timeout is set (for example, if no response within 0.1–0.2 seconds, it will consider the call failed for safety).
        
    5. Upon receiving the service response, extract the recommended velocity command (linear and angular velocity).
        
    6. Return this velocity command to `move_base`. `move_base` will then publish it on `/cmd_vel`.
        
- **Emergency Fallback:** If the service call fails (e.g., no response, or the optimizer reports failure via a flag in the response), the plugin triggers a safe fallback. The simplest fallback is to command **zero velocity** (stop the robot) to avoid uncontrolled motion. Optionally, parameters could allow a different behavior (like a last resort obstacle clearing rotation or reversion to a simpler planner), but by default a stop is safest. The plugin will log a warning if the PSDF-MPC solver fails and ensure the robot halts or slows significantly in such cases.
    
- **Goal Reached Check:** The plugin implements `isGoalReached()` to inform `move_base` when the local goal is achieved. This can be based on distance to the final goal from the global plan and whether the robot’s velocity is nearly zero. If the PSDF-MPC brings the robot within some tolerance of the goal, this returns true, signaling navigation completion.
    

Internally, the local planner plugin does not perform heavy computations; it mainly relays information and possibly does light preprocessing (like truncating the global plan for the local horizon and coordinate transforms). This keeps the real-time loop efficient. All optimization is delegated to the `psdf_ros` service.

### PSDF ROS Service Node (Python)

The `psdf_ros` node (`psdf_ros_node.py`) runs as a separate ROS node and provides the `/psdf_mpc` service. This is the "brain" of the local planner, running the PSDF-based MPC optimization. Key roles of this component:

- **Service Interface:** It advertises a ROS service named `/psdf_mpc` of type `PsdfMpc.srv`. When a request is received (containing the robot’s state and a reference path), the node will:
    
    1. Update the internal optimizer with the latest robot state (pose, velocity).
        
    2. Incorporate the provided reference trajectory for tracking (setting the desired path for the horizon).
        
    3. Ensure the obstacle data is up-to-date (the node maintains obstacles from sensor inputs; see below).
        
    4. Solve the MPC optimization problem to compute an optimal control sequence (or at least the first control action).
        
    5. Return the first control action as the commanded linear and angular velocity (in a Twist message), along with a success status.
        
- **PSDF-MPC Optimizer:** The node uses the PSDF library and an MPC solver (based on ACADOS) to compute the control. The optimization problem can be summarized as:
    
    - **Horizon & Dynamics:** Optimize over a finite horizon of N steps (e.g., N=15 steps, covering ~1.5 seconds ahead). The robot’s **differential drive dynamics** (x-dot = v_cosθ, y-dot = v_sinθ, θ-dot = ω) constrain the motion.
        
    - **Cost function:** Minimize tracking error to the reference trajectory and minimize control effort. Quadratic costs are defined by weight matrices Q (for state error) and R (for control input). A terminal cost weight `Q_f` is applied at the end of the horizon for final error. (These weights are configurable; by default Q = diag(50, 50, 1) for (x,y,θ) and R = diag(0.2, 0.05) for (v, ω), tuned for balanced tracking vs smooth control.)
        
    - **Constraints:**
        
        - **Dynamic limits:** Velocity and acceleration limits are enforced (e.g., v_min/v_max, ω_min/ω_max, and optionally acceleration constraints on v and ω, reflecting robot capabilities). For instance, default limits might be v ∈ [-0.5, 0.5] m/s and ω ∈ [-1.2, 1.2] rad/s, with acceleration limited to 0.2 m/s² and 0.8 rad/s².
            
        - **Collision avoidance:** The critical constraint is that the **PSDF value ≥ d_safe** for all predicted states along the horizon. Here, `d_safe` is a safety distance (default 0.001 m, essentially ensuring non-negativity of distance, but it can be set higher for more conservative clearance). The PSDF function computes the signed distance from the robot’s footprint to the nearest obstacle edge; enforcing `PSDF(robot_pose, obstacles) ≥ d_safe` ensures the robot stays at least `d_safe` away from any obstacle. This constraint is enforced at each time step of the horizon.
            
    - **Solver:** The optimization is solved using **ACADOS** (a high-performance optimal control solver) via its Python interface. The problem is formulated with CasADi symbolic expressions for dynamics and the PSDF constraint. Real-time iteration (SQP-RTI) is used for efficiency, and warm-starting is enabled so the previous solution helps initialize the next solve. Typical solve times on CPU are aimed to be well under 100 ms to meet the 10 Hz requirement (actual performance depends on scenario complexity, but the design target is ~10–20 ms per solve on a modern i7 or ~50–100 ms on a Jetson Xavier).
        
- **Obstacle Updater:** The `psdf_ros` node either internally contains or works closely with an obstacle detection module (as illustrated by `obstacle_detector.py`). This component:
    
    - Subscribes to one or more sensor or map topics to receive obstacle data. For example, it could subscribe to a 2D LiDAR scan (`sensor_msgs/LaserScan`), depth camera data, or a costmap topic. The exact source is configurable; what matters is that it produces **line segment representations** of obstacles.
        
    - Processes raw data into line segments and clusters them. Clustering is done by proximity so that each cluster corresponds to one obstacle or contiguous structure (for instance, a wall segment or a cluster of points forming an obstacle’s boundary). The module might use a **local planning window** (e.g., a 3m x 3m window around the robot) to consider only nearby obstacles for efficiency.
        
    - Expands or inflates obstacles by a safety margin if needed (the code supports creating a "cap" or convex hull around segments with a small margin, typically 0.05 m, to ensure the PSDF distance includes a buffer).
        
    - Outputs obstacles as **Edge Clusters**. Each cluster is essentially a set of line segments defined by their endpoints. The PSDF node maintains a list of these clusters, up to a maximum number _K_max_ (e.g., 20 clusters) and with each cluster having up to _E_max_ edges (e.g., 64 edges max). If more obstacles are detected than the maximum, the farthest or smallest clusters might be dropped (the system will log if obstacles are being truncated).
        
    - The PSDF internal representation (`psdfWrapper`) is updated with the obstacle clusters. This involves passing arrays of segment start and end points for each cluster to the PSDF library, which in turn updates internal tensors for fast distance computation. This update can be done each cycle or whenever new obstacle info arrives; it’s efficient due to vectorization.
        
- **Robot Footprint Geometry:** Upon startup, the PSDF node initializes the PSDF model with the robot’s footprint shape. The footprint must be a convex polygon (e.g., rectangle or circle approximated by a polygon). The vertices of this polygon are provided in the robot’s local frame (`base_link`). For example, a differential drive robot might use a rectangular footprint given by four corner points. The PSDF model uses these vertices to precompute certain constants (like edge normals of the robot shape) so that at runtime it can efficiently calculate distances. This means **accurate robot geometry is critical** – it should encompass any protrusions (like lidar or bumper) for safety. The footprint can be specified via a config file (e.g., `robot_footprint.yaml`) or it can reuse the footprint specified for the costmap (if move_base parameter `footprint` or `robot_radius` is set, the same can be loaded to PSDF to avoid inconsistencies).
    
- **Outputs:** After solving, the node outputs the first step of the optimal control sequence as a velocity command. This is typically a forward linear velocity and an angular velocity (since we consider a differential drive model). The command is given in the robot’s base frame (x-axis forward, z-axis angular velocity), consistent with ROS conventions for `/cmd_vel`. The service response includes this velocity as `geometry_msgs/Twist` (with an optional timestamp and frame in `TwistStamped`).
    
- **Failure Handling:** If the solver fails to find a feasible solution (which could happen if the goal is unreachable or obstacles block the path with no feasible gap), the node can respond with `success = false` and a zero velocity. It will log the failure and possibly the ACADOS status for debugging. The local planner plugin, upon seeing `success=false` or a timeout, will stop the robot as mentioned earlier. This ensures that any solver failure does not lead to unsafe movement.
    
- **Computational Considerations:** Running PyTorch and ACADOS in a Python node introduces computational load. The node is designed to remain within the 100 ms per cycle budget:
    
    - ACADOS performs the heavy optimization in C code under the hood.
        
    - The PSDF distance calculations are vectorized and can optionally be offloaded to GPU if a CUDA device is available (though the baseline is CPU). The code supports specifying `device: "cpu"` or `"cuda"` for the PSDF computations – by default it’s CPU to ensure compatibility and stability.
        
    - If using a GPU (future extension), the distance computations for all horizon steps can be parallelized on the GPU, but the overall benefit must be weighed against data transfer times. On a Xavier (with CUDA), initial tests would be needed to tune this. The default setting is CPU which has been sufficient for ~10 Hz in testing.
        
    - The node also attempts to warm-start the solver each cycle by reusing the previous solution trajectory. This significantly speeds up convergence, as the solution won't start from scratch each time (especially important for MPC, where successive solutions are similar).
        

In essence, the `psdf_ros` service node encapsulates all the complex math and ensures the rest of the ROS system sees a simple interface (service in, velocity out). This makes the advanced PSDF-MPC approach accessible to any ROS1 navigation stack.

## ROS Interfaces

This section defines the ROS interfaces provided by the PSDF-ROS package, including services, topics, and message types. All custom message/service types are defined in the `psdf_ros/msg` and `psdf_ros/srv` directories and are installed for use in ROS.

### Service: `PsdfMpc.srv` (`/psdf_mpc`)

The primary interface is the **PSDF-MPC service** used by the local planner plugin to request a velocity command. The service is defined as follows:

```
# PsdfMpc.srv - Service definition for PSDF-MPC local planner

# Request
geometry_msgs/PoseStamped   current_pose    # Robot's current pose in the planning frame (e.g., odom)
geometry_msgs/Twist         current_velocity # Robot's current velocity (in base frame axes, typically from odometry)
nav_msgs/Path               reference_path   # Desired reference path for the horizon (poses in planning frame)

---

# Response
geometry_msgs/TwistStamped  cmd_vel          # Commanded velocity to apply (in base_link frame)
bool                        success          # True if optimization succeeded, False if no valid command found

```


**Request fields:**

- `current_pose`: The robot’s pose at the current time. This should be given in the **local planning frame** (commonly `odom`). The orientation in PoseStamped (quaternion) is used to get the current heading θ.
    
- `current_velocity`: The robot’s current velocity. Only the relevant components are used (for a differential drive, linear.x and angular.z). This provides the MPC initial velocity (important for a dynamically feasible trajectory).
    
- `reference_path`: A short path that the MPC should try to follow over the next few seconds. This is given as a `nav_msgs/Path` (essentially a header and an array of PoseStamped). This path should also be expressed in the same frame as `current_pose` (e.g., `odom`). Typically, the plugin will derive this from the global plan: e.g., take the next N waypoints of the global plan in odom frame, or sample points along the global plan up to the horizon distance. If the global goal is very close (shorter than horizon), the reference_path can simply end at the goal. **Note:** The length of `reference_path` should ideally be N+1 poses (matching the horizon length N if possible) or can be fewer, in which case the final pose can be repeated for the remaining steps.
    

**Response fields:**

- `cmd_vel`: The velocity command computed by the MPC. This is given as a `geometry_msgs/TwistStamped` with the same frame as `current_pose` (usually `odom` or `base_link`). The linear and angular velocities in this twist are what should be applied immediately to the robot. By convention, if the frame is `odom`, the twist is understood as relative to the robot’s heading (since odom frame is usually aligned such that the robot’s base_link x-axis is in the direction of motion). To avoid confusion, this could alternatively be given in `base_link` frame; as an implementation detail, we ensure consistency in usage. (For simplicity, one can assume the linear.x in `cmd_vel` is forward speed and angular.z is rotation rate about the robot’s center, as usual for differential drive.)
    
- `success`: A boolean flag indicating if the optimizer found a valid solution. If `false`, `cmd_vel` may be omitted or zero. The local planner should interpret `false` as an indication to stop or perform a fallback maneuver.
    

The service is expected to be called at a regular rate (10 Hz or higher). Each call should complete quickly. If a call is still in progress when the next cycle comes, the local planner might skip that cycle or use the last known good command (though in our implementation we prefer to stop rather than use stale commands). It’s important that clients (like the plugin) set a reasonable timeout on the service call (e.g., 0.1s) to avoid waiting too long.

 

**Example usage (pseudo-code)** for the service in the C++ plugin:

```cpp
psdf_ros::PsdfMpc srv;
srv.request.current_pose = currentPoseStamped;    // e.g., from TF
srv.request.current_velocity = currentVelocity;
srv.request.reference_path = localPathSegment;
if (!psdf_mpc_client.call(srv)) {
    ROS_ERROR("PSDF MPC service call failed");
    cmd_vel = geometry_msgs::Twist(); // zero as fallback
} else if (!srv.response.success) {
    ROS_WARN("PSDF MPC optimization failed, stopping");
    cmd_vel = geometry_msgs::Twist(); // zero command
} else {
    cmd_vel = srv.response.cmd_vel.twist;
}

```

### Topics and Messages

**Obstacle Input Topics:** The PSDF-ROS node expects obstacle data in the form of line segments. 

For clarity and modularity, we document the **EdgeClusters message interface**, as it would be used if obstacle data were published from an external source:

```
# EdgeSegment.msg
# Defines a line segment by its two endpoints in 2D (z is ignored or 0)
float32 x1
float32 y1
float32 x2
float32 y2
```


```
# EdgeCluster.msg
# A cluster of obstacle edges (e.g., representing one object or contiguous obstacle)
EdgeSegment[] segments
```

```
# EdgeClusters.msg
# An array of clusters, with a header for frame and timestamp
std_msgs/Header header    # e.g., frame_id = "odom" (frame in which edge coordinates are given)
EdgeCluster[] clusters
```

In typical usage, `header.frame_id` would be set to the same local frame used for planning (e.g., "odom"). Each EdgeCluster in `clusters` contains one obstacle’s worth of segments. These messages allow flexible representation of obstacles beyond just points or circles – capturing the actual geometry for the PSDF calculation.

The PSDF node, upon receiving `EdgeClusters`, will update its internal PSDF model:

- It will take up to the first K clusters (K_max configurable, default ~20).
    
- For each cluster, take up to E_max segments (default ~64, though the message could carry more, extras will be ignored or downsampled).
    
- Internally convert each segment to the format needed (two endpoints A and B). The PSDF expects two arrays: A (start points of segments) and B (end points of segments) for each cluster.
    
- It will then call the PSDF wrapper to update these clusters in the distance field model. This operation is efficient (just copying into tensors) and is done either whenever a new message arrives or at least before each solve.
    

**Other Topics:**

- **`/cmd_vel`:** This is the standard ROS topic for velocity commands. The `move_base` node ultimately publishes the velocity here after getting it from the local planner. The PSDF-ROS node itself does _not_ publish to `/cmd_vel` (to avoid conflicts); it only returns the velocity via service. However, for testing purposes, one could run `psdf_ros` in a stand-alone mode (with a test client) and have it publish `/cmd_vel` directly. In integrated mode, though, we keep the output within move_base’s normal publishing.
    
- **Visualization topics:** (Optional) The PSDF-ROS node may publish some visualization aids:
    
    - A `nav_msgs/Path` of the optimized trajectory (`/psdf_ros/optimized_path`) for debugging in RViz.
        
    - `visualization_msgs/MarkerArray` for obstacle edges and robot footprint, to verify the PSDF sees obstacles correctly.
        
    - These are not required for operation but are useful for development and can be enabled via parameters.
        

**TF frames:** The node will use TF transforms to relate frames:

- It might listen to the transform between the planning frame (odom) and base frame to convert sensor data or pose data as needed.
    
- The local planner plugin obtains the robot pose via TF (`map`→`odom`→`base_link` chain) and should provide `current_pose` in odom. So both obstacles and robot state align in the `odom` coordinate system.
    

In summary, the key ROS interface provided by this package is the `/psdf_mpc` service and the custom messages for obstacle edges (if used). They allow the decoupling of the navigation logic from how obstacles are detected. A user could replace the obstacle source (e.g., use a vision system that outputs EdgeSegments) without changing the core planner.

## Configuration and Parameters

The behavior of PSDF-ROS is highly configurable. Parameters are set via ROS parameter server, typically loaded from YAML files in the `config/` directory. Below is a list of important parameters, along with their meaning and default values:

**MPC Planning Parameters:**

|Parameter|Description|Default Value|
|---|---|---|
|`horizon`|Number of discrete steps in the MPC horizon (N). Each step is of duration `dt`. A larger N increases lookahead but also computation.|**15** steps|
|`dt`|Time step for each horizon interval (seconds). Total horizon time = `horizon * dt`.|**0.1** s (thus 1.5 s horizon)|
|`Q`|State error weight matrix (3x3 for [x, y, θ]). Higher values penalize deviation from reference path. Specified as a list or diagonal entries.|**diag(50, 50, 1)**|
|`R`|Control effort weight matrix (2x2 for [v, ω]). Higher values penalize large control inputs or oscillations.|**diag(0.2, 0.05)**|
|`terminal_weight`|Terminal cost weight factor (applied to final state error). Can be used if wanting to enforce goal accuracy.|**1.0** (dimensionless)|
|`d_safe`|Safety distance (m) – minimum allowed distance from obstacles. The PSDF constraint enforces `distance >= d_safe`. Increase this for more conservative planning.|**0.001** m (effectively 0 margin by default)|
|`v_max` / `v_min`|Max and min linear velocity (m/s). `v_min` can be negative for allowing reverse motion.|**0.5** / **-0.5** m/s (forward/backward)|
|`omega_max` / `omega_min`|Max and min angular velocity (rad/s).|**1.2** / **-1.2** rad/s|
|`accel_lim` (`a_max`)|Maximum linear acceleration (m/s²). Also defines deceleration limit (assumed symmetric for simplicity). This constrains how quickly the linear velocity can change.|**0.2** m/s²|
|`omega_accel_lim` (`alpha_max`)|Maximum angular acceleration (rad/s²). Constrains rotational acceleration.|**0.8** rad/s²|
|`allow_reverse`|Boolean to allow moving backward if it helps avoid obstacles or track path. If false, v_min will effectively be 0.|**true** (since v_min is set to -0.5 by default, we allow some reverse)|
|`solver_type`|The type of nonlinear solver. Options: `SQP_RTI` (Real-Time Iteration), `SQP`, etc. `SQP_RTI` is typically used for fast reactive control.|**SQP_RTI**|
|`qp_solver`|QP solver used by ACADOS for the subproblem. Default is `PARTIAL_CONDENSING_HPIPM`.|**PARTIAL_CONDENSING_HPIPM**|
|`max_sqp_iter`|Max iterations for nonlinear solver (NLP). If using SQP_RTI, this might be 1 (since RTI does one iteration per cycle).|**20** iterations|
|`max_qp_iter`|Max iterations for QP solver per SQP step.|**50**|
|`tol`|Solver tolerance for convergence.|**1e-4**|

**Obstacle & PSDF Parameters:**

| Parameter                         | Description                                                                                                                                                                                                                              | Default Value                                    |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| `max_clusters` (`K_max`)          | Maximum number of obstacle clusters to consider. If more obstacles are present, the extras are ignored (or handled via some strategy like dropping farthest).                                                                            | **20** clusters                                  |
| `max_edges_per_cluster` (`E_max`) | Maximum edges per cluster. Extra edges may be dropped or merged. This defines the maximum complexity of a single obstacle shape.                                                                                                         | **64** edges (sufficient for most convex shapes) |
| `local_window_width`              | Width of the local obstacle window (if using a local window in obstacle detector), in meters. Only obstacles within ±width/2 in the lateral direction are considered.                                                                    | **3.0** m                                        |
| `local_window_height`             | Height of the local window (in direction of travel), in meters. Only obstacles within ±height/2 forward/back are considered.                                                                                                             | **3.0** m                                        |
| `obstacle_safety_margin`          | Inflation margin for obstacles (m). Edges may be offset outward by this distance when forming convex "caps" around sensor detections. This provides extra buffer.                                                                        | **0.05** m                                       |
| `obstacle_topic`                  | The topic name for incoming EdgeClusters if using external obstacle feed. E.g., `"/detected_edges"`. If empty or not set, the node might use internal mechanisms (like directly reading a costmap or LaserScan).                         | **""** (must be set to use external feed)        |
| `obstacle_topic_type`             | Type of obstacle input: "edges" for EdgeClusters, "costmap" for reading costmap, "laser" for LaserScan etc. This can alter how the node interprets the data.                                                                             | **"edges"** (if using EdgeClusters)              |
| `device`                          | Computation device for PSDF calculations. `"cpu"` or `"cuda"`. If CUDA is chosen but not available, it will fall back to CPU.                                                                                                            | **"cpu"**                                        |
| `cluster_selection`               | Strategy when #clusters > max_clusters. Options: "nearest" (keep closest N clusters), "largest" (keep largest obstacles), etc.                                                                                                           | **"nearest"** (implied strategy)                 |
| `frame_id/ global_frame`          | The global frame for the global plan (usually `"map"`). The plugin will transform the global plan from this frame to the local planning frame.                                                                                           | **"map"**                                        |
| `frame_id/ local_frame`           | The local planning frame in which the PSDF-MPC operates and obstacles are given. Typically the odometry frame (`"odom"`).                                                                                                                | **"odom"**                                       |
| `frame_id/ base_frame`            | The robot base frame (the frame of the footprint polygon).                                                                                                                                                                               | **"base_link"**                                  |
| `robot_footprint`                 | The robot’s footprint specification. This can be given as an array of points (e.g., `[[x1, y1], [x2, y2], ...]` in meters around the base_link origin). Alternatively, a radius can be given for circular footprint. **Must be convex.** | **None (must be provided)**                      |
| `emergency_stop_on_fail`          | Whether to enforce a full stop when the MPC fails or is infeasible. If false, the controller might attempt other measures (not recommended unless a secondary planner is available).                                                     | **true**                                         |
| `service_timeout`                 | Timeout for waiting on the solver service (in seconds). The plugin side uses this; if the service call exceeds this, it aborts and triggers fallback.                                                                                    | **0.2** s                                        |

These parameters are typically loaded via the `psdf_local_planner.yaml` and can be adjusted to tune performance:

- For example, increasing `horizon` to 20 (2 seconds) might improve foresight at the cost of computation.
    
- Adjusting `Q` and `R` changes behavior: larger Q makes the planner stick to the reference path more aggressively, larger R makes it smoother (less control effort, more deviation allowed).
    
- `d_safe` can be raised to, say, 0.1 m if the robot shape is an exact footprint and you want a 10 cm buffer.
    
- The velocity and acceleration limits should match the robot’s physical limits and what the low-level motor controllers can handle.
    
- If the environment has many obstacles, you might increase `max_clusters` (if computationally feasible) or reduce the local window size to ignore far obstacles.
    

**Launch-time Configuration:** The `psdf_mpc.launch.xml` (see below) will load these config files and set parameters accordingly. The user should verify the `frame_id` parameters match their TF tree (particularly, ensure `odom` frame is correct and continuous). Also, ensure the robot footprint is correctly set, either by copying from the `move_base` config or redefining it here.

## Launch File: `psdf_mpc.launch.xml`

The package provides a launch file to start the PSDF-MPC local planner alongside `move_base`. An example structure of this launch file is:

```xml
<!-- psdf_mpc.launch.xml: Launch PSDF-ROS local planner with move_base -->
<launch>
    <!-- Load planner and optimizer parameters -->
    <param file="$(find psdf_ros)/config/psdf_local_planner.yaml" command="load" />
    <param file="$(find psdf_ros)/config/robot_footprint.yaml" command="load" />

    <!-- Launch the PSDF ROS service node -->
    <node name="psdf_ros" pkg="psdf_ros" type="psdf_ros_node.py" output="screen">
        <!-- Example remapping or parameter if using an external obstacle topic: -->
        <!-- <remap from="detected_edges" to="/some/obstacle_provider/edges" /> -->
        <param name="obstacle_topic" value="detected_edges" />
        <param name="obstacle_topic_type" value="edges" />
        <param name="device" value="cpu" />  <!-- Use CPU for PSDF computations -->
    </node>

    <!-- Launch move_base with the PSDF local planner plugin -->
    <node name="move_base" pkg="move_base" type="move_base" output="screen">
        <!-- Use the psdf_local_planner as the base_local_planner -->
        <param name="base_local_planner" value="psdf_local_planner/PSDFLocalPlanner" />
        <!-- Other move_base params like global_planner, costmaps, etc., can be set here or in separate files -->
        <param name="controller_frequency" value="10.0" />
        <!-- move_base might also have its own footprint param; ensure consistency if so -->
    </node>
</launch>
```

**Explanation:**

- We first load configuration parameters from YAML files. This makes all the parameters (described in the previous section) available to the nodes.
    
- We then start the `psdf_ros` node (Python). It will advertise the service and start listening for obstacle messages. We can pass parameters or remaps here. For instance, if an external node is publishing obstacles to `"/lidar_edges"`, we could set `<param name="obstacle_topic" value="/lidar_edges"/>`. In this example, we assume a generic `detected_edges` topic and specify that the input type is "edges".
    
- Next, we launch `move_base`. We override its `base_local_planner` parameter to use our plugin. The value `"psdf_local_planner/PSDFLocalPlanner"` must match the name exported by the plugin (set in `package.xml` or a plugin XML in the package). This tells move_base to load our plugin instead of, say, DWAPlanner.
    
- We set `controller_frequency` to 10.0 Hz (the desired compute rate). This should match our expectations for PSDF-MPC performance. If the hardware allows, this could be higher (e.g., 20 Hz for more responsiveness), but 10 Hz is a safe starting point.
    
- All other aspects of `move_base` (like global planner, costmaps, recovery behaviors) remain unchanged. Our planner fits into the standard pipeline seamlessly.
    

Users can include `psdf_mpc.launch.xml` in their robot’s main launch file or run it directly to bring up navigation. It is also possible to run the `psdf_ros` node and the plugin in isolation for testing (e.g., feed it manual service calls or use in a simulation environment).

## Flow of Operation

This section describes a typical sequence of operations from start-up to navigation completion, illustrating how the components interact in real-time:

1. **Initialization:** The user launches the system using the provided launch file. The `psdf_ros` node starts:
    
    - It loads the robot footprint and initializes the PSDF model with those vertices.
        
    - It sets up the ACADOS optimizer with the given horizon, costs, and constraints. The obstacle constraint is added with the current set of obstacles (which might be none at startup, or static obstacles if provided). The solver is created and prepared.
        
    - It subscribes to the obstacle source (if configured). For example, it might subscribe to a `detected_edges` topic or directly to a LaserScan.
        
    - It advertises the `/psdf_mpc` service and waits for requests.
        
    - It logs readiness (e.g., "PSDF-MPC service ready. Waiting for requests...").  
        Meanwhile, `move_base` starts and loads our plugin:
        
    - The `PSDFLocalPlanner` plugin’s `initialize()` method is called. It creates a ROS service client bound to `/psdf_mpc`, and possibly a subscriber to odometry (if needed to get current velocity, though velocity could be gotten from the tf or other).
        
    - The plugin reads relevant parameters (like frame_ids, goal tolerance, etc.) and stores them.
        
    - It logs initialization success (e.g., "PSDFLocalPlanner initialized, waiting for global plan").
        
2. **Receiving a Goal:** The user sets a navigation goal (via RViz or an action call). The global planner (e.g., Navfn or global planner of choice) computes a global path from the current pose to the goal in the `map` frame. `move_base` receives this and calls `setPlan()` on our local planner plugin with the new path.
    
    - The plugin transforms this global path into the local frame (`odom`). This might involve using TF to get the current odom→map transform and applying it to each pose in the path. The transformed path (now relative to odom) is stored internally as the reference path to follow.
        
    - The plugin resets any internal state (for example, it might reset any cyclic buffer of previous commands, or flags that indicate a new goal).
        
    - The plugin will then proceed to track this path.
        
3. **Real-time Loop (each cycle):** `move_base` triggers the local planner at ~10 Hz:
    
    - The plugin obtains the current robot pose (through costmap’s getRobotPose or TF lookup of `base_link` in `odom`). It also obtains the current velocity (`geometry_msgs/Twist` from either the last command or odometry feedback if available).
        
    - It then determines the **reference_path segment** for this cycle. For instance, it may take the subset of the stored global path from the current robot position out to 1.5 seconds ahead. If the global path is longer than that, it truncates; if shorter, it uses the whole path. It ensures the first point of reference_path is the robot’s current pose (or very close) so that tracking error starts near zero.
        
    - The plugin constructs a `PsdfMpc.srv` request:
        
        - `current_pose` = current robot pose (in odom frame).
            
        - `current_velocity` = current vel (in base frame terms).
            
        - `reference_path` = nav_msgs/Path containing the poses for each future step (or fewer poses, the PSDF node will interpolate or handle missing points by assuming last pose as constant target).
            
    - It calls the service. The request is transported to the `psdf_ros` node.
        
    - In the `psdf_ros` node, upon receiving the request:
        
        - It grabs the latest obstacle data (likely already in memory from the last subscriber callback). If the obstacle topic is updating asynchronously, the PSDF node might have a recent set of clusters. It ensures these are fed into the PSDF model (calling `update_obstacles()` if needed). If obstacles have not changed since last time, this could be skipped or is very fast (just maintaining the same data).
            
        - It updates the optimizer's initial state (the current pose/velocity). The first state of the horizon is set to the current pose, and the initial guess for controls might be warm-started from last solution or a nominal controller.
            
        - It updates the reference trajectory for the horizon. If the provided `reference_path` has exactly N+1 points (matching horizon length), each stage i has reference state = that path point i. If there are fewer, it might interpolate or set the last given point for the remaining steps. Essentially, the optimizer now has a desired trajectory $\bar{x}_{0...N}$ to follow (where $\bar{x}_0$$ would be current state typically and $\bar{x}_N$ a future goal).
            
        - It triggers the solver to solve the MPC problem. This involves computing the cost and constraint Jacobians, solving a QP, etc., but ACADOS handles it under the hood. The PSDF constraint ensures any candidate trajectory that violates clearance is considered infeasible, so the optimizer will adjust velocities (even possibly deviating from the path) to maintain safety.
            
        - After solving, it retrieves the optimized control sequence ($u_0$ ... $u_{N-1}$). It picks the first control $u_0$ (this is the immediate command to send).
            
            - If `success`: The solver found a solution (likely, ACADOS status = 0). Then $u_0$ might be, for example, [v=0.3 m/s, ω=0.1 rad/s] meaning go forward slowly while slightly turning.
                
            - If solver failed: It sets `success=false`. It might still output a `cmd_vel` of zero or a very conservative guess, but typically we just indicate failure.
                
        - It responds to the service call with `cmd_vel` and the success flag.
            
    - Back in the plugin (client side): It receives the response (within the 0.1–0.2s timeout).
        
        - If `success=true`: it takes `cmd_vel` and passes it out of `computeVelocityCommands()` as the result.
            
        - If `success=false` or the service call itself errored/timed out: it will either command zero (stop) or use last known safe command. Our design choice is to stop to be safe. It also could set a flag to request a new global plan or trigger a recovery behavior if numerous cycles fail (this could tie into move_base’s recovery behavior mechanism by returning an error to move_base after several consecutive failures, prompting e.g. a rotate-in-place recovery).
            
    - `move_base` receives the Twist and publishes it on `/cmd_vel`. The robot’s base controller executes it, moving the robot.
        
    - This cycle repeats at the next tick: with the robot having moved a little, obstacles maybe changed, etc., always recomputing a fresh command.
        
4. **Obstacle Handling during motion:** Suppose a new obstacle appears (or moves) in front of the robot (e.g., a person steps in).
    
    - The sensor data (LaserScan or costmap) would reflect this. The obstacle detector (running continuously in background) will process the scan, cluster the new obstacle, and publish updated EdgeClusters (or update internal state).
        
    - The PSDF node’s subscriber callback receives this and updates its internal obstacle list. This could happen asynchronously between service calls. The next time the MPC service is called, it will be using the updated obstacle info.
        
    - The PSDF constraint in the optimization will cause the MPC to adjust the trajectory to avoid the new obstacle. For example, it might slow down or turn to a new heading to circumvent the person. Because the PSDF is differentiable, the MPC can even “see” the gradient of the distance and take smooth avoidance maneuvers rather than abrupt stops.
        
    - If the obstacle is too close and no path is feasible (say someone jumps right in front), the optimizer might fail (no feasible solution respecting `d_safe`). Then `success=false` triggers and the robot stops. Stopping itself ensures collision is avoided. The system could then wait or signal for a replanning or user intervention.
        
5. **Goal Reaching:** As the robot nears the final goal, the reference_path shrinks (often the global plan ends at the goal). The MPC will try to bring the robot exactly to the goal pose. Typically, a threshold for goal reached is something like distance < 0.1 m and angle < a few degrees and velocity nearly zero.
    
    - The plugin’s `isGoalReached()` will check the robot’s current pose against the goal after each cycle. Once within tolerance and if `move_base`’s oscillation/holonomic stopping criteria are satisfied, it returns true.
        
    - `move_base` then stops calling the local planner and declares the navigation goal succeeded.
        
    - The PSDF-ROS node may also detect it's at goal if reference_path is empty and could go idle, but primarily the coordination is done by move_base.
        
6. **Recovery and Fallback:** If the robot gets stuck or the PSDF local planner fails repeatedly, `move_base`’s logic might invoke recovery behaviors. For example, after X consecutive cycles where `computeVelocityCommands` returns false (or no progress), it could call a recovery behavior (like rotating in place or clearing costmaps). Our plugin tries to always return true (with zero velocity if needed to stop) to avoid triggering recovery unless truly necessary. However, in a scenario like being in a dead-end with no feasible path, such recovery triggers are desired. The PRD specifies that multi-robot is not supported and dynamic obstacles are handled reactively (not predicted), so in very tricky cases an external recovery or re-planning might be needed.
    

Throughout operation, timing is crucial. The target of ≥10 Hz means each cycle (including sensing, service call, compute, and command) should complete in under 100 ms. The PSDF node keeps track of solver timing internally (it can log each solve duration and perhaps publish statistics). If performance issues arise (e.g., solve takes 150 ms occasionally), one might adjust parameters (shorter horizon, simpler model, or use GPU if available). In practice, with the given defaults and moderate obstacle complexity, the system is expected to comfortably meet real-time requirements on recommended hardware.
## Frame Conventions and Coordinate Systems

Consistency in coordinate frames is vital for a correct functioning of the planner. We define the frames involved and how data is transformed:

- **Map frame (`map`):** A world-fixed frame used by the global planner. The global plan is typically provided in this frame (especially if using an external localization like SLAM or AMCL). The `move_base` global costmap also usually uses `map`. PSDF-ROS does not use `map` frame directly in calculations, but the plugin will convert the global plan from `map` to `odom`. The `map→odom` transform is assumed to be maintained by the localization system (representing odometric drift or reset).
    
- **Odometry frame (`odom`):** A world frame that drifts with the robot but is continuous and locally accurate. We use `odom` as the **planning frame** for local planning. All obstacle coordinates and robot poses for the MPC are expressed in the `odom` frame. This choice avoids large jumps or global resets affecting the local planner, and ensures that small integration errors in odometry don’t accumulate in a single run (since each cycle, we get a fresh pose in odom). The obstacle detector also uses `odom` for its internal local window origin (which is essentially the robot’s position in odom). If the user’s system does not provide an `odom` frame, `map` could be used as a substitute, but then one must be cautious about global shifts (the PSDF constraint would still work, but a global relocation could confuse the path following – however, that’s beyond local scope).
    
- **Base frame (`base_link`):** The robot’s base frame, attached to the robot. The footprint polygon is defined in this frame (e.g., if the robot is centered at (0,0) and front extends to +0.5m in x, etc., those coordinates are in base_link). The velocity commands (Twist linear.x, angular.z) are with respect to this frame’s axes (standard ROS convention: linear.x is forward along base_link x-axis). During the PSDF computation, the robot’s pose (x, y, θ in odom) is used to transform obstacle edges into the robot’s local frame internally so that the footprint polygon can be checked against them. Thus, the PSDF function effectively computes distances in the robot’s local coordinates.
    
- **Sensors frame(s):** For LIDAR or other sensors, there might be a sensor frame (e.g., `laser_frame`). The obstacle detector should take care of transforming sensor data into `odom` or `base_link` as needed to compute obstacle edges. For example, a 2D LIDAR scan can be projected into odom frame by knowing the robot pose. Or one could convert laser points to base_link and then to odom by adding the robot pose. In any case, by the time we form EdgeSegments, we want them in odom frame with a static timestamp.
    

**Transforms & Usage:**

- When `psdf_local_planner` receives a global plan in `map`, it transforms it: for each PoseStamped in the plan, it uses TF to get that pose in `odom`. It produces the `reference_path` in `odom` frame, and sets `reference_path.header.frame_id = "odom"`.
    
- The `current_pose` is obtained in `odom` frame directly (e.g., from the costmap or TF `odom→base_link`).
    
- The `psdf_ros` node knows the `reference_path.header.frame_id` and `current_pose.header.frame_id` (both should be "odom"). If there's a mismatch, it will try to transform, but ideally they match. It might assert that the incoming poses are in the configured `local_frame` and warn if not.
    
- The obstacle EdgeClusters, if coming from an external node, have `header.frame_id = "odom"`. If the obstacle detector is internal, it will naturally produce them in odom using the robot pose at time of detection.
    
- The output TwistStamped `cmd_vel` is given with `header.frame_id = "base_link"` (or "odom", depending on convention chosen). We prefer to interpret the command in the robot’s frame. Typically, move_base and robot controllers assume the Twist is in base_link coordinates (so linear.x is forward). To avoid confusion: we can set `cmd_vel.header.frame_id = "base_link"` explicitly. The local planner plugin or move_base doesn't actually use that frame_id, it just forwards the twist, but it’s good documentation.
    
- The coordinate convention for orientation is yaw (θ) in the plane, per ROS standard (quaternions in Pose, but effectively we deal with yaw angle for heading).
    
- All distances (like in the obstacle segments) are in meters in the respective frame coordinates.
    

By adhering to these conventions, we ensure that the geometry is consistent:

- The robot’s shape (in base_link) is correctly located in odom frame when checking collisions.
    
- The obstacles in odom frame are correctly related to robot’s odom pose.
    
- The global plan converted to odom aligns with actual obstacle positions in odom, which is critical if the map → odom drift is non-negligible (the local planner then handles small discrepancies, and the global planner can replan if drift accumulates).
    

We also note that **time synchronization** matters: The `PoseStamped` for current pose might come with a timestamp (though often one just uses the latest available transform). We assume quasi-instant sync between getting the pose and obstacle data. For fast-moving robots, one might consider using time tags and interpolation, but given 10 Hz and typical speeds, the lag is minor.

## Robot Footprint and Geometry

A correct robot footprint is central to PSDF’s accuracy:

- **Convex Polygon Footprint:** The robot is modeled as a convex polygon. Common choices are a rectangle encompassing the robot or an octagon/circle approximating it. Convexity is required because the PSDF algorithm implicitly assumes a convex shape for distance computation (it finds the _minimum distance_ from any point on the robot to any obstacle edge, which only equals the distance between convex shapes if both are convex). If a concave shape were used, distance might be miscomputed in concave recesses – hence not supported.
    
- **Specification:** The footprint can be specified in the `robot_footprint.yaml` as a list of points (x, y pairs). Example:
    ```yaml
robot_footprint:
  # points in base_link frame, making a rectangle 0.6m x 0.4m
  - [-0.3, -0.2]
  - [ 0.3, -0.2]
  - [ 0.3,  0.2]
  - [-0.3,  0.2]
     ```
    
    These should be in counter-clockwise order for consistency (the PSDF code expects CCW ordering of vertices). The polygon should be closed (last point connects back to first).
    
- **Automatic import:** Optionally, we could fetch the footprint from the ROS costmap params if the user already defined `footprint` for move_base. E.g., move_base sometimes has a param like `<param name="footprint" value="[[x1,y1],[x2,y2],...]" />`. Our plugin can retrieve that via the ROS param server on init and pass it to the PSDF node (via service or a latched topic or directly writing to a parameter the node reads). Implementation detail aside, we document that the user should ensure the same footprint is used for costmap and PSDF to avoid scenarios where costmap thinks an obstacle is farther/closer than it actually is to the robot shape.
    
- **Inflation vs PSDF safe distance:** In costmaps, one often inflates obstacles by a certain radius. With PSDF, we can largely rely on the actual geometry and safe distance. If the costmap is still used for global planning, it may be inflating obstacles. That’s fine; the local planner will just see a global plan that perhaps stands off obstacles a bit. We should clarify: the PSDF’s `d_safe` could play a similar role to inflation, adding an extra buffer on top of the actual shapes. Usually, to be safe, one might not inflate the global costmap too much (to allow tight moves) but use a small `d_safe` to enforce local clearance. Tuning of these should be documented in a user guide.

    
- **Verification:** We recommend verifying the footprint visually (the launch could publish a marker of the polygon) and ensuring that the PSDF distance outputs make sense (a unit test could place a single obstacle edge at a known offset and see if PSDF returns the expected distance minus footprint radius). For example, if the robot is a circle of radius 0.3m and an obstacle is at 2.0m from robot center, PSDF should return ~1.7m distance (2.0 - 0.3) if it’s a horizontal line in front.
    

In conclusion, the robot footprint is a required input for the PSDF-ROS package – it must be set correctly in configuration. The PSDF algorithm uses this polygon to accurately compute distances to obstacles, thereby ensuring that collision avoidance is performed with respect to the robot’s true shape, not just a point or overly conservative bounding box.

## Testing Plan

To ensure the PSDF-ROS local planner meets its requirements and is robust, a comprehensive testing strategy will be employed:

 

**1. Unit Tests (Offline Functional Tests):**

- _PSDF Function Unit Test:_ Write a test that directly calls the PSDF distance computation on known geometries. For example, place a single line segment obstacle at known positions and verify the signed distance value. Also test orientation: if the robot rotates, PSDF’s result should remain consistent (since distance is rotation-invariant for a symmetric footprint or properly accounting orientation for non-symmetric). This ensures the PSDF implementation is correct.
    
- _Optimizer Constraint Test:_ Create a simple scenario with a static obstacle and see if the solver honors `d_safe`. For instance, set up a scenario in code with an obstacle at (1,0) and a goal beyond it; verify that the optimizer either goes around or refuses to go straight through. Check that any violation of distance constraint leads to solver infeasibility (which is expected).
    
- _Service Call Simulation:_ Use a dummy client (could be a Python script or a ROS unittest node) to call `/psdf_mpc` with simple inputs. For example, no obstacles, start at (0,0) heading to (1,0). The expected output should be a forward velocity ~v_max and minimal turning. If we put an obstacle directly in front, expected behavior might be a turn or a stop (depending on if it can steer around within horizon). These functional tests help catch logical errors in how the service processes data.
    
- _Edge Cases:_ Test behavior at boundaries: extremely close obstacle (distance < d_safe right at start), extremely sharp turn needed (goal behind the robot), etc. The planner should either handle them or cleanly report failure (and not crash).
    

**2. Integration Tests (ROS, Simulation):**

- _Gazebo Simulation:_ Integrate PSDF-ROS in a Gazebo environment. Use a differential drive robot model with ROS control and sensors. Set up a few scenarios:
    
    - Straight line with no obstacles (to test basic tracking).
        
    - Obstacle avoidance: e.g., place a box between start and goal, and verify the robot goes around it smoothly. Compare to standard planners for baseline.
        
    - Dynamic obstacle: a moving person model crossing the path. Check the robot slows/stops as needed. (This might require a Gazebo plugin or manual movement of obstacle during test.)
        
    - Narrow passage: two obstacles making a corridor just big enough for the robot. Test that the robot can get through (if it is feasible) by hugging one side possibly. This tests the PSDF gradient in a tight scenario.
        
- _Logging and Analysis:_ Enable verbose logs or ROS bag recording. After simulation, analyze:
    
    - The `/cmd_vel` over time to see if velocities respect limits (no spikes).
        
    - The minimum distance to obstacles over time (can compute from logged robot position and obstacle positions).
        
    - The path taken vs the global plan (to see if it diverged reasonably when needed).
        
- _Performance measurement:_ In simulation (or on real hardware), measure the actual compute time. We can add timing output around the service call in the plugin. Ensure meeting 10 Hz; if not, adjust parameters and re-test.
    

**3. Bag Replay Testing:**

- If real sensor data is available (e.g., a recorded LaserScan and odometry from a real robot in an environment with obstacles), we perform a **bag replay test**. Run the `psdf_ros` node with the bagged data feeding into it (and perhaps a dummy local planner client that calls the service based on the bag’s odometry). This isolates the planner in a controlled but realistic data scenario. We can simulate a drive by feeding odometry from bag as the robot moves and see how planner would respond. This helps in tuning obstacle detection parameters and verifying the planner’s decisions on real sensor noise.
    
- A simpler approach is to record `EdgeClusters` messages from a run and then feed them in a loop while moving the robot (either simulation or a script moving a virtual robot). The planner’s responses can be checked.
    

**4. Hardware-in-the-Loop (HITL) / Real Robot Testing:**

- Start with a cautious test on a real robot in a controlled environment. For instance, in a lab with a few known obstacles (boxes or walls). Use a slow max speed to begin (e.g., 0.2 m/s) and a short goal. Observe behavior.
    
- Test scenario: set a goal beyond an obstacle and see if the robot navigates around. Because this is a new planner, have a e-stop ready and possibly limit movements. Verify that if the solver fails (maybe by artificially raising d_safe during a run to simulate no solution), the robot stops promptly.
    
- Testing on different hardware: If possible, test on both an Intel NUC (i7) and a Jetson Xavier to ensure no platform-specific issues (especially with ACADOS binary or PyTorch).
    
- Multi-run consistency: run the same path multiple times to see if results are repeatable or if there's variability (MPC can sometimes have slight variations).
    

**5. Regression Tests:**

- After any code changes, re-run critical tests (like the Gazebo scenarios and unit tests) to ensure no performance degradation or new collisions introduced.
    
- Maintain a set of scenario configurations and expected outcomes (could be as simple as “goal reached without collision” flags) to check.
    

**6. Testing Constraints and Limits:**

- Test the limits of `max_clusters` and `max_edges_per_cluster`. For example, simulate 30 obstacles around the robot and ensure the system handles the first 20 and ignores the rest gracefully (no crashes or huge slowdowns).
    
- Test with a very complex shape obstacle (like a circle approximated by 100 small segments) to see if E_max truncation yields acceptable distance approximation. If an obstacle is truncated, ensure at least it covers the nearest part (since we likely sort by cluster distance when updating).
    
- If available, test on a scenario with multiple robots (not that we support multi-robot planning, but to ensure one robot’s obstacles can include the other as a moving obstacle if needed, treated just as dynamic obstacle).
    

**HITL vs. SITL Clarification:** By "Hardware-in-the-loop", we mean testing with the real robot’s sensors and possibly in a partial simulation (for instance, wheel commands go to robot but robot is lifted or wheels on blocks to see commanded behavior without actual movement). This can be used to validate the system’s outputs without risking motion. Once confident, actual driving tests (fully hardware) can proceed.

 

The testing plan ensures that all requirements (real-time performance, collision avoidance, etc.) are verified in practice. It’s recommended to document the results of each test scenario, noting any failures or required tuning. Over time, this will provide confidence in the system’s reliability and guide further improvements.

