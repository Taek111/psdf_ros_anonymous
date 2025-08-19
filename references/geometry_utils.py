from abc import ABCMeta

import casadi as ca
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import polytope as pt
from pypoman import compute_polytope_vertices
import torch


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
    """[Genral polytope shape]"""

    def __init__(self, mat_A, vec_b):
        self.mat_A = mat_A
        self.vec_b = vec_b
        self.points = pt.extreme(pt.Polytope(mat_A, vec_b))

    @classmethod
    def convex_hull(self, points):
        """Convex hull of N points in d dimensions as Nxd numpy array"""
        P = pt.reduce(pt.qhull(points))
        return PolytopeRegion(P.A, P.b)
    
    

    def get_convex_rep(self):
        # TODO: Move this change into constructor instead of API here
        return self.mat_A, self.vec_b.reshape(self.vec_b.shape[0], -1)

    def get_plot_patch(self):
        return patches.Polygon(self.points, closed=True, linewidth=1, edgecolor="k", facecolor="r")
    
    def get_ccw_vertices(self):
        """Returns the vertices of the polytope in counter-clockwise order"""
        vertices = compute_polytope_vertices(self.mat_A, self.vec_b)
        centroid = np.mean(vertices, axis=0)  # e.g. [0., 0.] (위 예시는 정사각형이라 0,0)
        def angle_from_centroid(pt):
            dx = pt[0] - centroid[0]
            dy = pt[1] - centroid[1]
            return np.arctan2(dy, dx)
        
        vertices_ccw = sorted(vertices, key=angle_from_centroid)

        return np.array(vertices_ccw)  # Ensure numpy array is returned


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