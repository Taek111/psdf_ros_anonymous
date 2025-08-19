import datetime
import casadi as ca
import numpy as np
import torch
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import time
import os
import rospy
from psdf import PSDF
from psdf_wrapper import PSDFWrapper
import l4casadi as l4c
from utils import polygon_to_edges
from l4casadi.realtime import RealTimeL4CasADi
import yaml

class PSDFOptimizerConfig:
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

    @staticmethod
    def from_yaml(file_path: str):
        """Load configuration from a YAML file."""
        cfg = PSDFOptimizerConfig()
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            for k, v in data.items():
                if hasattr(cfg, k):
                    setattr(cfg, k, v)
        except Exception as e:
            print(f"Warning: failed to load optimizer config from {file_path}: {e}")
        return cfg


class PSDFOptimizer:
    """(Deprecated) Alias kept for backward compatibility. Use PSDFOptimizer instead."""
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
        self.psdf_wrapper = None  # PSDFWrapper instance
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
        print(f"set state: {self.state._x}, {self.state._u}")

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
            # Use terminal reference key explicitly
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

            # If no obstacles provided, clear clusters and return early
            if len(all_clusters_A) == 0:
                self.psdf_wrapper.clear_clusters()
                print("No obstacles provided; cleared PSDF clusters")
                return

            # Update psdfWrapper with all clusters
            if len(all_clusters_A) <= self.psdf_wrapper.K_max:
                self.psdf_wrapper.update_edge_clusters(all_clusters_A, all_clusters_B)
            else:
                # Use only first K_max clusters
                self.psdf_wrapper.update_edge_clusters(all_clusters_A[:self.psdf_wrapper.K_max], all_clusters_B[:self.psdf_wrapper.K_max])
        else:
            print("Warning: PSDF wrapper not initialized, skipping obstacle update")                

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
            x_ws, u_ws = system.nominal_safe_controller(
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
        # Ensure the initial state constraint is updated at solve time
        # (match behavior in references/psdf_optimizer.py)
        if self.state is not None:
            try:
                self.solver.set(0, "lbx", self.state._x)
                self.solver.set(0, "ubx", self.state._x)
            except Exception as e:
                # Fallback: rely on x0 set during setup if direct bounds are unsupported
                print(f"Warning: failed to set stage-0 state bounds via solver: {e}")

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
            
        
        return AcadosSolution(self.solver, self.N, self.variables, status)

    def solve(self, state, reference_trajectory):
        """
        Solve the optimization problem with given state and reference trajectory.
        
        Args:
            state: Current state vector [x, y, theta]
            reference_trajectory: Reference trajectory for MPC
            
        Returns:
            tuple: (success, u_opt, info) where:
                success: bool indicating if solve was successful
                u_opt: optimal control inputs [v, omega]
                info: additional solver information
        """
        try:
            rospy.loginfo(f"[MPC_OPTIMIZER] Starting solve with state: {state}")
            rospy.loginfo(f"[MPC_OPTIMIZER] Reference trajectory shape: {reference_trajectory.shape if hasattr(reference_trajectory, 'shape') else len(reference_trajectory)}")
            
            # Set current state
            self.set_state(state)
            rospy.loginfo("[MPC_OPTIMIZER] State set successfully")
            
            # Set reference trajectory
            self.set_reference_trajectory(reference_trajectory)
            rospy.loginfo("[MPC_OPTIMIZER] Reference trajectory set successfully")
            
            # Solve the optimization problem
            rospy.loginfo("[MPC_OPTIMIZER] Calling solve_nlp...")
            solution = self.solve_nlp()
            rospy.loginfo("[MPC_OPTIMIZER] solve_nlp completed")
            
            # Extract optimal control inputs
            u_traj = solution.get_input_trajectory()
            rospy.loginfo(f"[MPC_OPTIMIZER] Input trajectory extracted: shape={u_traj.shape if u_traj is not None else None}")
            
            if u_traj is not None and len(u_traj) > 0:
                # Return first control input
                v_opt = float(u_traj[0, 0])
                omega_opt = float(u_traj[1, 0])
                u_opt = [v_opt, omega_opt]
                
                # Check solver status
                stats = solution.stats()
                success = stats.get('return_status', 'failure') == 'success'
                
                info = {
                    'solver_time': getattr(self, 'solver_times', [0])[-1] if hasattr(self, 'solver_times') else 0,
                    'status': stats.get('return_status', 'unknown')
                }
                
                rospy.loginfo(f"[MPC_OPTIMIZER] Solve successful: v={v_opt}, omega={omega_opt}")
                return success, u_opt, info
            else:
                rospy.logwarn("[MPC_OPTIMIZER] Solve failed: No solution found")
                return False, [0.0, 0.0], {'error': 'No solution found'}
                
        except Exception as e:
            rospy.logerr(f"[MPC_OPTIMIZER] Exception in solve: {str(e)}")
            import traceback
            traceback.print_exc()
            return False, [0.0, 0.0], {'error': str(e)}


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
            vertices = system._geometry.get_ccw_vertices() # Get vertices in counter-clockwise order
            
            print(f"vertices.shape: {vertices.shape}")
            # PSDFWrapper 초기화
            self.psdf_wrapper = PSDFWrapper(
                verts=torch.tensor(vertices, dtype=torch.float32, device=device),
                E_max=E_max,
                K_max=K_max,
                device=device
            )
            
            print(f"PSDFWrapper initialized with vertices shape: {vertices.shape}")
            
        else:
            raise ValueError("System must have _geometry attribute")
   
        # 배치 Jacobian 계산은 GPU에서, acados 내부에서는 파라미터화 된 1차 근사만 사용
        self.ped_model = RealTimeL4CasADi(self.psdf_wrapper,
                                          approximation_order=1,
                                          device=self.device)

class AcadosSolution:
    """Helper class to mimic casadi solution interface"""
    
    def __init__(self, solver, N, variables, status):
        self.solver = solver
        self.N = N
        self.variables = variables
        self.status = status
    
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
        return {
            "return_status": "success" if int(self.status) == 0 else "failure",
            "status_code": int(self.status)
        }
