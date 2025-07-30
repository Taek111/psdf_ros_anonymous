import datetime
import casadi as ca
import numpy as np
import torch
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import time
import os
from models.psdf import PSDF
from models.psdf_wrapper import psdfWrapper
from models.dd import DifferentialDriveRectangleGeometry
import l4casadi as l4c
from models.geometry_utils import polygon_to_edges
from models.obstacle_detector import LocalWindowObstacleDetector
from l4casadi.realtime import RealTimeL4CasADi

class PSDFOptimizerParam:
    def __init__(self):
        self.horizon = 20
        self.mat_Q = np.diag([50.0, 50.0, 1.0])  # Reduced from 100.0
        self.mat_R = np.diag([0.2, 0.05])  # Reduced from 20.0
        
        self.terminal_weight = 1.0  # 10.0
        
        # SQP specific parameters
        self.tf = 0.1 * self.horizon  # total time horizon
        self.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # qp solver to be used
        self.hessian_approx = 'GAUSS_NEWTON'  # Hessian approximation
        self.integrator_type = 'ERK'  # explicit Runge-Kutta
        self.nlp_solver_type = 'SQP_RTI'  # SQP solver
        self.qp_solver_iter_max = 50  # maximum iterations for QP solver
        self.nlp_solver_max_iter = 20  # maximum iterations for NLP solver
        self.tol = 1e-4  # convergence tolerance
        
        # Input constraints
        self.vmin, self.vmax = -0.7, 0.7
        self.omegamin, self.omegamax = -1.2, 1.2
        
        # Input derivative constraints
        self.amin, self.amax = -0.3, 0.3
        self.omegadot_min, self.omegadot_max = -0.8, 0.8

        # Safety distance parameter
        self.d_safe = 0.0001  # minimum safe distance to obstacles
        
        # Soft-constraint parameters
        # If `use_soft_constraint` is True, the obstacle avoidance constraint will be softened
        # by introducing a non-negative slack variable that is penalised in the cost function
        # with quadratic weight `slack_weight`.
        self.use_soft_constraint = False
        self.slack_weight = 1e16
        
        # Obstacle detection parameters
        self.detection_window_width = 4.0   # local window width [m]
        self.detection_window_height = 4.0  # local window height [m]
        self.detection_safety_margin = 0.05 # safety margin for obstacle caps [m]
        self.detection_frequency = 10.0     # obstacle detection frequency [Hz]


class PSDFOptimizer:
    def __init__(self, variables=None, costs=None, dynamics_opt=None):
        self.ocp = None
        self.solver = None
        self.solver_times = []
        self.state = None
        self.reference_trajectory = None
        self.N = None
        self.nx = None
        self.nu = None
        self.variables = {}
        self.costs = {} if costs is None else costs
        self.dynamics_opt = dynamics_opt
        self.psdf_wrapper = None  # psdfWrapper instance
        self._is_initialized = False
        
        # Local window obstacle detector
        self.obstacle_detector = None
        self.detection_frequency = 10.0  # Hz
        self.last_detection_time = 0.0
        
        # Use common JSON filename - will be cleaned up after each test
        self.json_filename = "acados_ocp.json"
        
        # Store files to cleanup
        self._temp_files = []

    def cleanup(self):
        """Clean up temporary files and reset state"""
        try:
            # Clean up temporary files
            for file_path in self._temp_files:
                if os.path.exists(file_path):
                    os.remove(file_path)
            
            # Clean up JSON file
            if os.path.exists(self.json_filename):
                os.remove(self.json_filename)
                
            # Clean up acados generated directories
            cleanup_dirs = [
                "c_generated_code",
            ]
            for dir_name in cleanup_dirs:
                if os.path.exists(dir_name) and os.path.isdir(dir_name):
                    import shutil
                    shutil.rmtree(dir_name, ignore_errors=True)
            
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
        
        # Reset internal state
        self.ocp = None
        self.solver = None
        self.solver_times = []
        self.psdf_wrapper = None
        self.ped_model = None
        self._is_initialized = False

    def __del__(self):
        """Destructor to ensure cleanup"""
        self.cleanup()

    def reset(self):
        """Reset optimizer state for new test"""
        self.cleanup()

    def set_state(self, state):
        self.state = state

    def set_reference_trajectory(self, reference_trajectory):
        """Set the reference trajectory for the optimizer"""
        self.reference_trajectory = reference_trajectory
        if self.solver is not None and reference_trajectory is not None:
            # Set stage costs
            for i in range(self.N):
                yref = np.zeros(self.nx + self.nu)
                yref[:self.nx] = reference_trajectory[i, :]
                self.solver.set(i, "yref", yref)
            
            # Set terminal cost
            yref_e = reference_trajectory[-1, :]
            self.solver.set(self.N, "yref", yref_e)

    def create_model(self, param):
        """Create the acados model for the robot dynamics"""
        # State and input dimensions
        nx = 3  # x, y, theta
        nu = 2  # v, omega
        
        # Symbolic variables
        x = ca.MX.sym('x', nx)
        xdot = ca.MX.sym('xdot', nx)
        u = ca.MX.sym('u', nu)
        
        # Dynamics - differential drive robot
        f_expl = ca.vertcat(
            u[0] * ca.cos(x[2]),  # x_dot = v * cos(theta)
            u[0] * ca.sin(x[2]),  # y_dot = v * sin(theta)
            u[1]                  # theta_dot = omega
        )
        
        # Create acados model
        model = AcadosModel()
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = 'differential_drive_psdf'
        
        return model

    def setup_ocp(self, param, reference_trajectory):
        """Setup the optimal control problem"""
        self.ocp = AcadosOcp()
        
        # Create model
        model = self.create_model(param)
        self.ocp.model = model
        
        # Dimensions
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        N = param.horizon
        self.N = N
        self.nx = nx
        self.nu = nu
        
        # Set dimensions
        self.ocp.dims.N = N
        
        # Set cost
        self.ocp.cost.cost_type = 'LINEAR_LS'
        self.ocp.cost.cost_type_e = 'LINEAR_LS'
        
        # Weight matrices
        Q = param.mat_Q
        R = param.mat_R
        
        # Cost matrices for reference tracking and input
        self.ocp.cost.W = np.block([[Q, np.zeros((nx, nu))],
                                   [np.zeros((nu, nx)), R]])
        # Terminal cost (weighted by terminal_weight)
        self.ocp.cost.W_e = Q * param.terminal_weight
        
        # Reference tracking
        self.ocp.cost.Vx = np.zeros((nx + nu, nx))
        self.ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        self.ocp.cost.Vu = np.zeros((nx + nu, nu))
        self.ocp.cost.Vu[nx:, :] = np.eye(nu)
        self.ocp.cost.Vx_e = np.eye(nx)
        
        # Set reference trajectory
        yref_init = np.zeros((nx + nu,))
        if reference_trajectory.size > 0:
            yref_init[:nx] = reference_trajectory[0, :]
        self.ocp.cost.yref = yref_init
        self.ocp.cost.yref_e = reference_trajectory[-1, :]
        
        # Set constraints
        # Input constraints
        self.ocp.constraints.lbu = np.array([param.vmin, param.omegamin])
        self.ocp.constraints.ubu = np.array([param.vmax, param.omegamax])
        self.ocp.constraints.idxbu = np.array([0, 1])
        
        # Initial state constraint
        if self.state is not None:
            self.ocp.constraints.x0 = self.state._x
        else:
            self.ocp.constraints.x0 = np.zeros(nx)
        
        # Add input derivative constraints (rate constraints)
        # This requires setting up a custom constraint in acados
        # For simplicity, we'll use soft constraints via cost function
        
        # Set options
        self.ocp.solver_options.qp_solver = param.qp_solver
        self.ocp.solver_options.hessian_approx = param.hessian_approx
        self.ocp.solver_options.integrator_type = param.integrator_type
        self.ocp.solver_options.nlp_solver_type = param.nlp_solver_type
        self.ocp.solver_options.qp_solver_iter_max = param.qp_solver_iter_max 
        self.ocp.solver_options.nlp_solver_max_iter = param.nlp_solver_max_iter
        self.ocp.solver_options.tol = param.tol
        # Set warm start options
        self.ocp.solver_options.qp_solver_warm_start = True  
        self.ocp.solver_options.nlp_solver_warm_start_first_qp = True  
        # Set prediction horizon
        self.ocp.solver_options.tf = param.tf
        
        # Store reference trajectory
        self.reference_trajectory = reference_trajectory

        

    def create_solver(self):
        # RealTimeL4CasADi 는 외부 동적 라이브러리가 없으므로 링크 옵션을 설정하지 않는다.
        from l4casadi.realtime import RealTimeL4CasADi
        if hasattr(self, 'ped_model') and self.ped_model is not None and not isinstance(self.ped_model, RealTimeL4CasADi):
            # L4CasADi dynamic library 정보를 acados 컴파일러 옵션에 추가
            self.ocp.solver_options.model_external_shared_lib_dir = self.ped_model.shared_lib_dir
            self.ocp.solver_options.model_external_shared_lib_name = self.ped_model.name
       
        """Create the acados solver"""
        self.variables["x"] = "x"
        self.variables["u"] = "u"
        self.solver = AcadosOcpSolver(self.ocp, json_file=self.json_filename)
        self._temp_files.append(self.json_filename)

    def update_obstacles(self, obstacles_geo): 
        """Update the obstacles in the PSDF wrapper (legacy method)"""
        if self.psdf_wrapper is not None:
            all_clusters_A = []
            all_clusters_B = []
            for obs_geo in obstacles_geo:
                edgesA, edgesB = polygon_to_edges(obs_geo)
                all_clusters_A.append(edgesA)
                all_clusters_B.append(edgesB)

            # Update psdfWrapper with all clusters
            if len(all_clusters_A) <= self.psdf_wrapper.K_max:
                self.psdf_wrapper.update_edge_clusters(all_clusters_A, all_clusters_B)
            else:
                # Use only first K_max clusters
                self.psdf_wrapper.update_edge_clusters(all_clusters_A[:self.psdf_wrapper.K_max], all_clusters_B[:self.psdf_wrapper.K_max])
        else:
            print("Warning: PSDF wrapper not initialized, skipping obstacle update")                

    def update_obstacles_with_detection(self, obstacles, robot_pose, force_update=False):
        """
        Local window detection 기반 obstacle 업데이트
        
        Args:
            obstacles: 전체 obstacle 리스트
            robot_pose: 현재 로봇 pose [x, y, theta]
            force_update: 시간 체크 무시하고 강제 업데이트
        """
        if self.obstacle_detector is None or self.psdf_wrapper is None:
            print("Warning: Obstacle detector or PSDF wrapper not initialized")
            return
            
        current_time = time.time()
        
        # Check if update is needed based on frequency
        if not force_update and (current_time - self.last_detection_time) < (1.0 / self.detection_frequency):
            return
            
        # Perform local window detection
        try:
            clusters_A, clusters_B = self.obstacle_detector.detect_local_obstacles(robot_pose, obstacles)
            if clusters_A and clusters_B:
                # Update psdfWrapper with detected clusters
                self.psdf_wrapper.update_edge_clusters(clusters_A, clusters_B)
            else:
                # No obstacles in local window - clear existing clusters
                self.psdf_wrapper.clear_clusters()
                
            self.last_detection_time = current_time
            
        except Exception as e:
            print(f"Error in local obstacle detection: {e}")
            # Fallback to legacy method
            self.update_obstacles(obstacles)

    def should_update_detection(self) -> bool:
        """Check if obstacle detection should be updated based on frequency"""
        current_time = time.time()
        return (current_time - self.last_detection_time) >= (1.0 / self.detection_frequency)

    def set_detection_frequency(self, frequency_hz: float):
        """Set the obstacle detection update frequency"""
        self.detection_frequency = max(0.1, frequency_hz)  # Minimum 0.1 Hz
        print(f"Obstacle detection frequency set to {self.detection_frequency} Hz")

    def configure_detection_from_param(self, param):
        """Configure obstacle detection parameters from PSDFOptimizerParam"""
        if self.obstacle_detector is not None:
            # Update obstacle detector parameters
            self.obstacle_detector.update_detection_params(
                window_width=param.detection_window_width,
                window_height=param.detection_window_height,
                safety_margin=param.detection_safety_margin
            )
            
            # Update detection frequency
            self.set_detection_frequency(param.detection_frequency)
            
            print(f"Obstacle detection configured: "
                  f"window={param.detection_window_width}x{param.detection_window_height}m, "
                  f"margin={param.detection_safety_margin}m, freq={param.detection_frequency}Hz")                
           

    def add_obstacle_avoidance_constraint(self, param, system, obstacles_geo):
        
        self.update_obstacles(obstacles_geo)

        # Add obstacle avoidance constraint using PED model
        if self.ped_model is not None and self.ocp is not None:
            # Define constraint function: h(x) = sdf(x, y, theta) - d_safe >= 0
            x = self.ocp.model.x  # state variables [x, y, theta]
            
            sdf_value = self.ped_model(x)
            constraint_expr = sdf_value - param.d_safe
            
            # Set up nonlinear constraint
            self.ocp.constraints.constr_type = 'BGH'
            
            # --- expose real-time Taylor parameters as model parameter vector ---
            p_sym = self.ped_model.get_sym_params()  # (np, 1)
            self.ocp.model.p = p_sym
            self.ocp.dims.np = p_sym.shape[0]
            # Provide default parameter values (will be overwritten each iteration)
            self.ocp.parameter_values = np.zeros((p_sym.shape[0], ))

            # For path constraints (applied at each time step)
            nh = 1  # number of constraints (scalar sdf)
            self.ocp.dims.nh = nh

            # Set constraint expression (depends on p)
            self.ocp.model.con_h_expr = constraint_expr

            # Set constraint bounds: h(x) >= 0 (hard lower bound)
            self.ocp.constraints.lh = np.array([0.0])
            self.ocp.constraints.uh = np.array([1e8])  # effectively no upper bound

            # ------------------------------------------------------------------
            # Soft-constraint handling using slack variables in acados
            # ------------------------------------------------------------------
            if getattr(param, "use_soft_constraint", False):
                # Number of soft constraints (lower bound violation only)
                ns = nh
                self.ocp.dims.ns = ns      # total number of slacks
                self.ocp.dims.nsh = nh     # slacks for nonlinear path constraints

                # Indices of the constraints that are softened (0-based)
                self.ocp.constraints.idxsh = np.arange(nh, dtype=np.int64)

                # Slack bounds (0 <= s <= large)
                self.ocp.constraints.lsh = np.zeros(ns)
                self.ocp.constraints.ush = np.ones(ns) * 1e8

                # Quadratic cost on slack variables
                slack_w = float(getattr(param, "slack_weight", 1e4))
                self.ocp.cost.Zl = np.diag([slack_w] * ns)
                self.ocp.cost.Zu = np.diag([slack_w] * ns)
                # Linear term (optional)
                self.ocp.cost.zl = np.zeros(ns)
                self.ocp.cost.zu = np.zeros(ns)

                print(f"Obstacle avoidance constraint softened with slack weight = {slack_w}")
            else:
                print(f"Added HARD obstacle avoidance constraint with d_safe = {param.d_safe}")
        else:
            print("Warning: PED model not initialized, skipping obstacle avoidance constraint")

    def add_warm_start(self, param, system):
        """Add warm start based on nominal safe controller"""
        if self.solver is None:
            return
        
        # Get warm start trajectory from nominal safe controller
        try:
            x_ws, u_ws = system._dynamics.nominal_safe_controller(
                self.state._x, 0.1, self.state._u[0], -1.0, 1.0
            )
            print("x_ws, u_ws :", x_ws, u_ws)
            
            # Set warm start for all horizons
            for i in range(self.N):
                # Set state warm start
                self.solver.set(i, "x", x_ws)
                # Set control warm start
                self.solver.set(i, "u", u_ws)
                
            # Set final state warm start
            self.solver.set(self.N, "x", x_ws)
            
        except Exception as e:
            print(f"Error in warm start: {e}")

    def setup(self, param, system, reference_trajectory, obstacles):
        """Setup the complete optimization problem"""
        self.set_state(system._state)

        if not self._is_initialized:
            print("Setting up PSDF optimizer...")
            # Setup the OCP Problem
            self.setup_ocp(param, reference_trajectory)
            self.initialize_ped_model(system, obstacles, E_max=100, device="cpu") # Set the PED model in the OCP
            # Configure obstacle detection from parameters
            self.configure_detection_from_param(param)
            self.add_obstacle_avoidance_constraint(param, system, obstacles)
            # Create the acados solver
            self.create_solver()
            self.add_warm_start(param, system)
            self._is_initialized = True
            
        
        else:
            # Set the reference trajectory
            self.set_reference_trajectory(reference_trajectory)
            # Obstacle update is now handled in the control loop (e.g., test_nmpc.py)
            # before this setup method is called.

    def solve_nlp(self):
        """Solve the NLP using SQP"""
        start = time.time()
        # Ensure the initial state constraint is updated
        if self.state is not None:
            self.solver.set(0, "lbx", self.state._x)
            self.solver.set(0, "ubx", self.state._x)
        
        # ------------------------------------------------------------------
        # Update real-time Taylor parameters for each stage (batch evaluation)
        # ------------------------------------------------------------------
        if self.ped_model is not None:
            # Gather current guessed states (warm start or previous solution)
            x_guess = np.stack([self.solver.get(i, "x") for i in range(self.N + 1)], axis=0)
            params_batch = self.ped_model.get_params(x_guess)  # (N+1, np)
            # Set parameters for each stage
            for i in range(self.N):
                self.solver.set(i, "p", params_batch[i])
            self.solver.set(self.N, "p", params_batch[-1])

        
        status = self.solver.solve()
        end = time.time()
        solve_time = end - start
        
        # Record solver time
        self.solver_times.append(solve_time)
        print("solver time: ", solve_time)
        
        if status != 0:
            print(f"Acados solver failed with status {status}")
            
        
        return AcadosSolution(self.solver, self.N, self.variables)


    def initialize_ped_model(self, system, obstacles, E_max=100, K_max = 20, device="cpu"):
        """
        시스템의 geometry로부터 psdfWrapper 초기화 및 obstacle detector 설정
        
        Args:
            system: robot system with geometry
            obstacles: initial obstacles list
            E_max: maximum number of edges to handle
            K_max: maximum number of clusters to handle
            device: torch device ("cpu" or "cuda")
        """
        self.device = device

        if hasattr(system, '_geometry'):
            geometry = system._geometry._geometries[0]  # Get the first geometry
            vertices = geometry._region.get_ccw_vertices() # Get vertices in counter-clockwise order
            
            print(f"vertices.shape: {vertices.shape}")
            # psdfWrapper 초기화
            self.psdf_wrapper = psdfWrapper(
                verts=torch.tensor(vertices, dtype=torch.float32, device=device),
                E_max=E_max,
                K_max=K_max,
                device=device
            )
            
            print(f"psdfWrapper initialized with vertices shape: {vertices.shape}")
            
        else:
            raise ValueError("System must have _geometry attribute")
        
        # Initialize obstacle detector with parameters from config
        # Note: param is not passed to this method, so we use default values
        # These can be set later via set_detection_params()
        self.obstacle_detector = LocalWindowObstacleDetector(
            window_width=4.0,       # Will be updated from param
            window_height=4.0,      # Will be updated from param
            safety_margin=0.05,     # Will be updated from param
            max_clusters=K_max,
            max_edges_per_cluster=10,
            device=device
        )
        print(f"LocalWindowObstacleDetector initialized with {self.obstacle_detector.window_width}x{self.obstacle_detector.window_height}m window")
        
        # 배치 Jacobian 계산은 GPU에서, acados 내부에서는 파라미터화 된 1차 근사만 사용
        self.ped_model = RealTimeL4CasADi(self.psdf_wrapper,
                                          approximation_order=1,
                                          device=self.device)


class AcadosSolution:
    """Helper class to mimic casadi solution interface"""
    
    def __init__(self, solver, N, variables):
        self.solver = solver
        self.N = N
        self.variables = variables
    
    def value(self, var_expr):
        """Get the value of a variable expression"""
        if var_expr == "x" or (isinstance(var_expr, str) and var_expr == "x"):
            return np.stack([self.solver.get(i, "x") for i in range(self.N + 1)], axis=1)
        elif var_expr == "u" or (isinstance(var_expr, str) and var_expr == "u"):
            return np.stack([self.solver.get(i, "u") for i in range(self.N)], axis=1)
        else:
            raise NotImplementedError("Only 'x' and 'u' supported in AcadosSolution.value")
    
    def get_state_trajectory(self):
        """Get the optimized state trajectory"""
        return self.value("x")
    
    def get_input_trajectory(self):
        """Get the optimized input trajectory"""
        return self.value("u")
    
    def stats(self):
        """Get solver statistics"""
        return {"return_status": "success" if self.solver.status == 0 else "failure"}


