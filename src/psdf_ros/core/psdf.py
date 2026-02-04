# psdf.py
#
# Polygon-Set Distance Field (PSDF)
# ---------------------
# Computes the signed distance between a **convex** robot footprint
# and **multiple edge-clusters** (obstacles) in a completely
# branch-free, differentiable pipeline.
#
# Shape convention (PyTorch-style):
#   K : # clusters (obstacles)          ≲ 32
#   E : max # edges per cluster         ≲ 64   (pad + mask)
#   B : # robot poses in a batch        ≲ 128
#   m : # footprint vertices  (fixed)
#
#   A , B   (K,E,2)  : start / end point of every edge
#   mask     (K,E)   : edge validity   (True = use, False = ignore)
#   poses      (B,3) : [x, y, θ]  (world frame)
#
#   All math is vectorised to   (B,K,E,…)  then
#       amin(dim=2) → per-cluster SDF
#       amin(dim=1) → min over all clusters
#
# 2025 - Taekwon Ga


import torch
import torch.nn.functional as F
from torch import nn, Tensor


class PSDF(nn.Module):
    r"""Poly-Edge Distance Network (cluster-aware)."""

    # --------------------------------------------------------------------- #
    # construction – identical to the original single-obstacle version
    # --------------------------------------------------------------------- #
    def __init__(self, verts: Tensor, eps: float = 1e-8):
        """
        verts (m,2) : CCW convex robot footprint vertices in **robot frame**
        eps         : small constant to avoid div-by-zero
        """
        super().__init__()

        V = verts.clone().detach()          # (m,2)
        m = V.shape[0]

        S  = torch.roll(V, -1, 0) - V                       # (m,2) edge vector
        LS = (S ** 2).sum(1, keepdim=True) + eps            # (m,1) |S|² + ε

        n  = F.normalize(torch.stack([-S[:, 1], S[:, 0]], 1), dim=1)  # (m,2)
        c  = -(n * V).sum(-1)                                # (m,)

        # ⇢ fixed footprint caches
        self.register_buffer("V",  V)        # (m,2)
        self.register_buffer("S",  S)        # (m,2)
        self.register_buffer("LS", LS)       # (m,1)
        self.register_buffer("n",  n)        # (m,2)
        self.register_buffer("c",  c)

        proj = V @ n.T
        self.register_buffer("poly_min", proj.amin(0))       # (m,)
        self.register_buffer("poly_max", proj.amax(0))       # (m,)

        self.inf = 1e12     # a *large* positive number for masking

    # --------------------------------------------------------------------- #
    # helper : point → segment squared distance       (broadcast-friendly)
    # --------------------------------------------------------------------- #
    @staticmethod
    def _p2seg_sq(P: Tensor, A: Tensor, v: Tensor, vL2: Tensor) -> Tensor:
        """
        Squared distance from point(s) *P* to segment(A, A+v).
        Shapes broadcast as long as the trailing “…,2” match.
        """
        u = ((P - A) * v).sum(-1) / vL2.squeeze(-1).clamp_min(1e-12)   # ∈ℝ
        t = u.clamp(0, 1)[..., None]                                    # Lerped 0–1
        Q = A + t * v
        return (P - Q).pow(2).sum(-1)                                   # dist²


    @staticmethod
    def _ray_inside(A_loc: Tensor, B_loc: Tensor, mask: Tensor) -> Tensor:
        """
        Odd–even rule (ray-casting) to decide if the point (0,0) lies
        inside a polygon given by *masked* segments (B,K,E,2).

        Returns:
            inside  (B,K)  True → (0,0) is inside that cluster
        """
        Ay, By = A_loc[..., 1], B_loc[..., 1]
        Ax, Bx = A_loc[..., 0], B_loc[..., 0]

        # 광선이 edge 와 y 높이 교차하는지
        crosses = ((Ay > 0) ^ (By > 0))                      # (B,K,E)
        # 교차점 x 좌표
        x_int   = Ax + (-Ay) * (Bx - Ax) / (By - Ay + 1e-12)
        crosses = crosses & (x_int > 0)                      # ray +x
        crosses = crosses & mask.unsqueeze(0)                # padding 제거

        # 홀짝 판정: odd → inside
        return (crosses.sum(-1) & 1).bool()                  # (B,K)
    # --------------------------------------------------------------------- #
    # forward
    # --------------------------------------------------------------------- #
    def forward(
        self,
        A: Tensor,               # (K,E,2)
        B: Tensor,               # (K,E,2)
        mask: Tensor,            # (K,E)   (bool)
        poses: Tensor,           # (B,3)
    ) -> Tensor:                 # →  (B,)  signed distance
        Bsz, K, E = poses.size(0), *A.shape[:2]
        m = self.V.size(0)

        # 1) world → robot-local transform for every pose ------------------
        cos, sin = poses[:, 2].cos(), poses[:, 2].sin()
        R = torch.stack([torch.stack([cos, sin], 1),
                         torch.stack([-sin,  cos], 1)], 1)          # (B,2,2)

        # translate
        A_rel = A.unsqueeze(0) - poses[:, :2].view(Bsz, 1, 1, 2)   # (B,K,E,2)
        B_rel = B.unsqueeze(0) - poses[:, :2].view(Bsz, 1, 1, 2)
        # rotate – flatten (K·E) for efficient bmm then reshape back
        A_loc = torch.bmm(
            A_rel.view(Bsz, -1, 2), R.transpose(1, 2)
        ).view(Bsz, K, E, 2)
        B_loc = torch.bmm(
            B_rel.view(Bsz, -1, 2), R.transpose(1, 2)
        ).view(Bsz, K, E, 2)
        A_loc_masked = A_loc.masked_fill(~mask.unsqueeze(0).unsqueeze(-1), self.inf)
        B_loc_masked = B_loc.masked_fill(~mask.unsqueeze(0).unsqueeze(-1), self.inf)
        v_obs  = B_loc - A_loc                         # (B,K,E,2)
        L2_obs = (v_obs ** 2).sum(-1, keepdim=True) + 1e-8   # (B,K,E,1)

        # ------------------- distance part (separation) ------------------- #
        P_v = self.V.view(1, 1, 1, m, 2)                        # (1,1,1,m,2)

        d_v_sq = self._p2seg_sq(
            P_v,
            A_loc.unsqueeze(3),       # (B,K,E,1,2)
            v_obs.unsqueeze(3),       # (B,K,E,1,2)
            L2_obs.unsqueeze(3),      # (B,K,E,1,1)
        )                             # → (B,K,E,m)

        # mask padded edges
        d_v_sq = d_v_sq.masked_fill(~mask.unsqueeze(0).unsqueeze(-1), self.inf)
        d_v_sq = d_v_sq.amin(2).amin(2)                       # (B,K)

        # obstacle end-points  → footprint edges -------------
        Pts   = torch.cat([A_loc, B_loc], 2)                  # (B,K,2E,2)
        mask2 = torch.cat([mask, mask], dim=1)          # (K, 2E)
        d_e_sq = self._p2seg_sq(
            Pts.unsqueeze(3),                                 # (B,K,2E,1,2)
            self.V.view(1, 1, 1, m, 2),                       # (1,1,1,m,2)
            self.S.view(1, 1, 1, m, 2),                       # (1,1,1,m,2)
            self.LS.view(1, 1, 1, m, 1),                      # (1,1,1,m,1)
        )                                                     # → (B,K,2E,m)

        d_e_sq = d_e_sq.masked_fill(~mask2.unsqueeze(0).unsqueeze(-1), self.inf)
        d_e_sq = d_e_sq.amin(3).amin(2)                       # (B,K)

        sep_dist = torch.minimum(d_v_sq, d_e_sq).clamp_min(1e-12).sqrt()  # (B,K)
        # beta = 50.0
        # sep_dist = -torch.logsumexp(
        #     -beta * sep_dist.unsqueeze(1).clamp_min(1e-12),  # (B,1,K)
        #     2
        # ) / beta                                            # (B,K)        

        # ------------------- overlap part (penetration) ------------------- #
        # (a) polygon-edge normals ----------------------------------------
        ends_proj = (Pts @ self.n.T)                          # (B,K,2E,m)

        mask_proj = mask2.unsqueeze(0).unsqueeze(-1)          # (1,K,2E,1)
        
        ends_min = ends_proj.masked_fill(~mask_proj, self.inf).amin(2)  # (B,K,m)
        ends_max = ends_proj.masked_fill(~mask_proj, -self.inf).amax(2)  # (B,K,m)
        ov_poly = torch.minimum(
            ends_max - self.poly_min,                         # (B,K,m)
            self.poly_max - ends_min,                         # (B,K,m)
        )                                                     # (B,K,m)

        # (b) segment normals -------------------------------------------------
        edge_mask = mask.unsqueeze(0)                             # (1,K,E)  True=valid

        # 1) n_seg 계산 (invalid edge의 v=0 => n_seg=0) 그대로 OK
        v_obs_masked = v_obs.masked_fill(~edge_mask.unsqueeze(-1), 0.0)
        n_seg = F.normalize(
            torch.stack([-v_obs_masked[..., 1], v_obs_masked[..., 0]], -1),
            dim=-1, eps=1e-12)                                    # (B,K,E,2)
        
        # 2) 로봇 폴리곤 투영 (poly_min_seg / poly_max_seg) – 기존 그대로
        proj_poly_seg = (n_seg @ self.V.T)                        # (B,K,E,m)
        proj_poly_seg_masked = proj_poly_seg.masked_fill(~edge_mask.unsqueeze(-1),  self.inf)
        poly_min_seg = proj_poly_seg_masked.amin(-1)   # +∞  (무효축)
        proj_poly_seg_masked = proj_poly_seg.masked_fill(~edge_mask.unsqueeze(-1), -self.inf)
        poly_max_seg = proj_poly_seg_masked.amax(-1)   # -∞  (무효축)
        
       # 3) **클러스터 전체 정점** 투영

        mask_all  = mask.unsqueeze(0).unsqueeze(2)              # (1,K,1,2E)

        proj_all  = (n_seg.unsqueeze(3) * A_loc_masked.unsqueeze(2)).sum(-1)  # (B,K,E,E)
        seg_min = proj_all.masked_fill(~mask_all,  self.inf).amin(-1)    # (B,K,E)
        seg_max = proj_all.masked_fill(~mask_all, -self.inf).amax(-1)    # (B,K,E)
       
        # 4) SAT overlap on segment normals
        ov_seg = torch.minimum(seg_max - poly_min_seg,
                            poly_max_seg - seg_min)            # (B,K,E)
        ov_seg = ov_seg.masked_fill(~edge_mask, self.inf)
        
        # (c) aggregate ----------------------------------------------------
    
        all_ov   = torch.cat([ov_poly, ov_seg], dim=2)        # (B,K,m+E)
        
        eps_sat = 1e-6  
        separated = (all_ov < -eps_sat).any(2)                     # (B,K) 
        separated_poly = (ov_poly < -eps_sat).any(2)               # (B,K)
        separated_seg  = (ov_seg  < -eps_sat).any(2)               # (B,K)
        
        beta = 100.0
        penetration = -torch.logsumexp(
            -beta * all_ov.clamp_min(0), 2
        ) / beta                                             # (B,K)

        inside = self._ray_inside(A_loc, B_loc, mask)     # (B,K)
        separated  = separated & (~inside)

        signed_cluster = torch.where(separated, sep_dist, torch.where(inside, -sep_dist, -penetration)) # (B,K)                                                  # (B,K)
        # final: min over K clusters ----------------------------
        return signed_cluster.amin(1)                         # (B,)

