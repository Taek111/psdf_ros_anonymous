from abc import ABCMeta

import casadi as ca
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import torch
from pypoman import compute_polytope_vertices
import polytope as pt
import math


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))

class ConvexRegion2D:
    __metaclass__ = ABCMeta

    def get_convex_rep(self):
        raise NotImplementedError()

    def get_plot_patch(self):
        raise NotImplementedError()


class RectangleRegion(ConvexRegion2D):
    """[Rectangle shape]"""

    def __init__(self, left, right, down, up):
        self.left = left
        self.right = right
        self.down = down
        self.up = up

    def get_convex_rep(self):
        mat_A = np.array([[-1, 0], [0, -1], [1, 0], [0, 1]])
        vec_b = np.array([[-self.left], [-self.down], [self.right], [self.up]])
        return mat_A, vec_b
    
    def get_ccw_vertices(self):
        # Returns the vertices of the rectangle in counter-clockwise order
        return np.array([
            [self.left, self.down],
            [self.right, self.down],
            [self.right, self.up],
            [self.left, self.up],
        ])
    
    def get_plot_patch(self):
        return patches.Rectangle(
            (self.left, self.down),
            self.right - self.left,
            self.up - self.down,
            linewidth=1,
            edgecolor="k",
            facecolor="r",
            alpha=0.4
        )


class PolytopeRegion(ConvexRegion2D):
    """[General polytope shape]"""

    def __init__(self, mat_A, vec_b):
        self.mat_A = mat_A
        self.vec_b = vec_b
        self.points = pt.extreme(pt.Polytope(mat_A, vec_b))

    @classmethod
    def convex_hull(cls, points):
        """Convex hull of N points in d dimensions as Nxd numpy array"""
        P = pt.reduce(pt.qhull(points))
        return cls(P.A, P.b)
    
    def get_convex_rep(self):
        return self.mat_A, self.vec_b.reshape(self.vec_b.shape[0], -1)

    def get_plot_patch(self):
        return patches.Polygon(self.points, closed=True, linewidth=1, edgecolor="k", facecolor="r")
    
    def get_ccw_vertices(self):
        """Returns the vertices of the polytope in counter-clockwise order"""
        if len(self.points) == 0:
            return np.array([])
            
        centroid = np.mean(self.points, axis=0)
        def angle_from_centroid(pt):
            dx = pt[0] - centroid[0]
            dy = pt[1] - centroid[1]
            return np.arctan2(dy, dx)
        
        vertices_ccw = sorted(self.points, key=angle_from_centroid)

        return np.array(vertices_ccw)  # Ensure numpy array is returned

class State:
    def __init__(self, x, u):
        self._x = x
        self._u = u


class DifferentialDriveSystem:
    """A class that combines differential drive dynamics and states"""
    
    def __init__(self, x, u=np.array([0.0, 0.0]), vertices=None):
        """
        Initialize the differential drive system
        
        Args:
            x: Initial state [x, y, yaw]
            u: Initial control input [v, w]
            vertices: Vertices of the robot geometry for PolytopeRegion
        """
        self._state = State(x, u)
        if vertices is not None:
            self._geometry = PolytopeRegion.convex_hull(np.array(vertices))
        else:
            self._geometry = None
    
    @staticmethod
    def forward_dynamics(x, u, timestep):
        """
        Return updated state in a form of `np.ndnumpy`
        states : x, y, yaw
        action : v, w
        """
        x_next = np.zeros(shape=(3,), dtype=float)
        x_next[0] = x[0] + u[0] * math.cos(x[2]) * timestep
        x_next[1] = x[1] + u[0] * math.sin(x[2]) * timestep
        x_next[2] = x[2] + u[1] * timestep
        return x_next
    
    @staticmethod
    def forward_dynamics_opt(timestep):
        """Return updated state in a form of `ca.SX`
        states : x, y, yaw
        action : v, w
        """
        x_symbol = ca.SX.sym("x", 3)
        u_symbol = ca.SX.sym("u", 2)
        x_symbol_next = x_symbol[0] + u_symbol[0] * ca.cos(x_symbol[2]) * timestep
        y_symbol_next = x_symbol[1] + u_symbol[0] * ca.sin(x_symbol[2]) * timestep
        theta_symbol_next = x_symbol[2] + u_symbol[1] * timestep
        state_symbol_next = ca.vertcat(x_symbol_next, y_symbol_next, theta_symbol_next)
        return ca.Function("DifferentialDrive_dynamics", [x_symbol, u_symbol], [state_symbol_next])
    
    @staticmethod
    def nominal_safe_controller(x, timestep, v_last, amin, amax):
        """
        Return updated state using nominal safe controller in a form of `np.ndnumpy`
        Make the velocity input that can be stopped in a timestep
        """
        u_nom = np.zeros(shape=(2,))
        a_nom = np.clip(-v_last / timestep, amin, amax)
        u_nom[0] = v_last + a_nom * timestep
        return DifferentialDriveSystem.forward_dynamics(x, u_nom, timestep), u_nom
    
    @staticmethod
    def safe_dist(timestep, v_last, amax, dist_margin):
        """Return a safe distance outside which to ignore obstacles"""
        safe_ratio = 1.25
        brake_min_dist = (abs(v_last) + amax * timestep) ** 2 / (2 * amax) + dist_margin
        return safe_ratio * brake_min_dist + abs(v_last) * timestep + 0.5 * amax * timestep ** 2
    
    def translation(self):
        """Get translation component of state"""
        return np.array([[self._x[0]], [self._x[1]]])
    
    def rotation(self):
        """Get rotation component of state"""
        return np.array(
            [
                [math.cos(self._x[2]), -math.sin(self._x[2])],
                [math.sin(self._x[2]), math.cos(self._x[2])],
            ]
        )
    
    def get_state(self):
        """Get the current state"""
        return self._x
    
    def update(self, unew, timestep):
        """Update the system state"""
        xnew = self.forward_dynamics(self._x, unew, timestep)
        self._x = xnew
        self._u = unew


class AckermannSystem:
    """Ackermann (bicycle) model with direct steering control."""

    def __init__(self, x, u=np.array([0.0, 0.0]), wheelbase=1.0, steering_limits=None, vertices=None):
        self.wheelbase = float(max(wheelbase, 1e-3))
        self.steering_limits = steering_limits or (-0.6, 0.6)
        state_vec = np.array(x, dtype=float)
        control_vec = np.array(u, dtype=float)
        self._state = State(state_vec, control_vec)
        if vertices is not None:
            self._geometry = PolytopeRegion.convex_hull(np.array(vertices))
        else:
            self._geometry = None

    def clamp_steering(self, delta):
        return _clamp(delta, self.steering_limits[0], self.steering_limits[1])

    def forward_dynamics(self, x, u, timestep):
        x_next = np.zeros(shape=(3,), dtype=float)
        v = float(u[0])
        delta = self.clamp_steering(float(u[1]))
        yaw_rate = v / self.wheelbase * math.tan(delta)
        x_next[0] = x[0] + v * math.cos(x[2]) * timestep
        x_next[1] = x[1] + v * math.sin(x[2]) * timestep
        x_next[2] = x[2] + yaw_rate * timestep
        return x_next

    def forward_dynamics_opt(self, timestep):
        x_symbol = ca.SX.sym("x", 3)
        u_symbol = ca.SX.sym("u", 2)
        v = u_symbol[0]
        delta = u_symbol[1]
        yaw_rate = v / self.wheelbase * ca.tan(delta)
        x_next = x_symbol[0] + v * ca.cos(x_symbol[2]) * timestep
        y_next = x_symbol[1] + v * ca.sin(x_symbol[2]) * timestep
        theta_next = x_symbol[2] + yaw_rate * timestep
        state_symbol_next = ca.vertcat(x_next, y_next, theta_next)
        return ca.Function("Ackermann_dynamics", [x_symbol, u_symbol], [state_symbol_next])

    def nominal_safe_controller(self, x, timestep, v_last, amin, amax):
        u_nom = np.zeros(shape=(2,))
        a_nom = np.clip(-v_last / max(timestep, 1e-3), amin, amax)
        u_nom[0] = v_last + a_nom * timestep
        u_nom[1] = self.clamp_steering(0.0)
        x_next = self.forward_dynamics(x, u_nom, timestep)
        return x_next, u_nom

    def translation(self):
        return np.array([[self._state._x[0]], [self._state._x[1]]])

    def rotation(self):
        return np.array(
            [
                [math.cos(self._state._x[2]), -math.sin(self._state._x[2])],
                [math.sin(self._state._x[2]), math.cos(self._state._x[2])],
            ]
        )

    def get_state(self):
        return self._state._x

    def update(self, unew, timestep):
        xnew = self.forward_dynamics(self._state._x, unew, timestep)
        v = float(unew[0])
        delta = self.clamp_steering(float(unew[1]))
        self._state = State(xnew, np.array([v, delta], dtype=float))


def get_dist_point_to_region(point, mat_A, vec_b):
    """Return distance between a point and a convex region"""
    opti = ca.Opti()
    # variables and cost
    point_in_region = opti.variable(mat_A.shape[-1], 1)
    cost = 0
    # constraints
    constraint = ca.mtimes(mat_A, point_in_region) <= vec_b
    opti.subject_to(constraint)
    dist_vec = point - point_in_region
    cost += ca.mtimes(dist_vec.T, dist_vec)
    # solve optimization
    opti.minimize(cost)
    option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
    opti.solver("ipopt", option)
    opt_sol = opti.solve()
    # minimum distance & dual variables
    dist = opt_sol.value(ca.norm_2(dist_vec))
    if dist > 0:
        lamb = opt_sol.value(opti.dual(constraint)) / (2 * dist)
    else:
        lamb = np.zeros(shape=(mat_A.shape[0],))
    return dist, lamb


def get_dist_region_to_region(mat_A1, vec_b1, mat_A2, vec_b2):
    opti = ca.Opti()
    # variables and cost
    point1 = opti.variable(mat_A1.shape[-1], 1)
    point2 = opti.variable(mat_A2.shape[-1], 1)
    cost = 0
    # constraints
    constraint1 = ca.mtimes(mat_A1, point1) <= vec_b1
    constraint2 = ca.mtimes(mat_A2, point2) <= vec_b2
    opti.subject_to(constraint1)
    opti.subject_to(constraint2)
    dist_vec = point1 - point2
    cost += ca.mtimes(dist_vec.T, dist_vec)
    # solve optimization
    opti.minimize(cost)
    option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
    opti.solver("ipopt", option)
    opt_sol = opti.solve()
    # minimum distance & dual variables
    dist = opt_sol.value(ca.norm_2(dist_vec))
    if dist > 0:
        lamb = opt_sol.value(opti.dual(constraint1)) / (2 * dist)
        mu = opt_sol.value(opti.dual(constraint2)) / (2 * dist)
    else:
        lamb = np.zeros(shape=(mat_A1.shape[0],))
        mu = np.zeros(shape=(mat_A2.shape[0],))
    return dist, lamb, mu


def polygon_to_edges(polytope_region):
    """
    Convert a PolytopeRegion to edge endpoints A and B tensors.
    
    Args:
        polytope_region: PolytopeRegion object
        
    Returns:
        tuple: (edgesA, edgesB) where each is a torch tensor of shape (N, 2)
               representing the start and end points of each edge
    """
    # Get vertices in counter-clockwise order
    vertices = polytope_region.get_ccw_vertices()
    vertices = np.array(vertices)
    
    # Create edges by connecting consecutive vertices
    num_vertices = len(vertices)
    edgesA = []
    edgesB = []
    
    for i in range(num_vertices):
        # Current vertex as start point
        start_point = vertices[i]
        # Next vertex as end point (wrap around to first vertex for last edge)
        end_point = vertices[(i + 1) % num_vertices]
        
        edgesA.append(start_point)
        edgesB.append(end_point)
    
    # Convert to torch tensors
    edgesA = torch.tensor(edgesA, dtype=torch.float32)
    edgesB = torch.tensor(edgesB, dtype=torch.float32)
    
    return edgesA, edgesB
