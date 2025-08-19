# Refactoring Plan for `scripts/mpc_optimizer.py`

Author: Cascade AI – 2025-07-31

## 1. Objectives
1. Replace hallucinated or ad-hoc logic in `scripts/mpc_optimizer.py`.
2. Ensure behaviour, API, and mathematical formulation match `references/psdf_optimizer.py`.
3. Preserve ROS integration points used by `psdf_ros_node.py` (parameters, topics, service calls).
4. Maintain code clarity, modularity, and testability.

---

## 2. Current State Summary
| Aspect | `scripts/mpc_optimizer.py` (current) | `references/psdf_optimizer.py` (gold-standard) |
| --- | --- | --- |
| Optimizer core | Custom CasADi MPC with partial PSDF | Fully-featured PSDF MPC using Acados & utilities |
| Safety constraints | Fragmented/implicit, some missing | Explicit PSDF constraint with `psdfWrapper` |
| Obstacle handling | Manual distance checks | `LocalWindowObstacleDetector` + soft/ hard constraints |
| Parameter handling | Hard-coded & duplicated | Centralised `PSDFOptimizerParam` class |
| API exposed to node | Inconsistent method names | Stable methods: `setup`, `set_state`, `solve_nlp`, etc. |

Main gaps: missing classes (`PSDFOptimizerParam`, obstacle detector), no warm-start, different variable naming, and missing soft-constraint support.

---

## 3. Target Architecture
1. **Configuration Class** – Introduce `PSDFOptimizerConfig` (same fields as `PSDFOptimizerParam`), loaded from a YAML file by `psdf_ros_node.py` and passed into `MPCOptimizer` during initialization.
2. **MPC Optimizer Class** (`MPCOptimizer`) – Thin subclass/ wrapper around `PSDFOptimizer` to keep file naming.
   * Import `PSDFOptimizer` from `references` or re-implement identical logic.
   * Retain ROS-specific glue (e.g., debug prints, message conversions).
3. **Public API**
   ```python
   class MPCOptimizer:
       def __init__(self, config: PSDFOptimizerConfig): ...
       def setup(self, system, reference, obstacles): ...
       def set_state(self, state): ...
       def solve(self): -> (x_traj, u_traj)
       def reset(self): ...
   ```
4. **Constraint & Cost Definition** – Direct copy of `add_obstacle_avoidance_constraint`, cost weights, horizon.
5. **Testing hooks** – Built-in unit test using pytest to compare solutions of both optimizers on random scenarios.

---

## 4. Refactoring Steps
1. **Baseline backup** – move existing `mpc_optimizer.py` to `mpc_optimizer_legacy.py` (keep git history).
2. **Copy reference logic** – copy contents of `references/psdf_optimizer.py` → new `mpc_optimizer.py`.
3. **Rename classes/functions** – s/PSDFOptimizer/MPCOptimizer/ where exposed externally.
4. **Prune unused code** – remove GUI, logging, file-IO sections not needed by ROS node.
5. **Interface alignment** – ensure `psdf_ros_node.py` calls match new signatures; adapt node if required.
6. **Configuration integration** – load YAML into `PSDFOptimizerConfig` via `psdf_ros_node.py` and pass to `MPCOptimizer`.
7. **Add tests**
   * Unit test: identical initial state & obstacles → both optimizers produce same cost within 1e-3.
   * Integration test: run node in simulation with simple scenario, ensure no exceptions.
8. **Documentation** – update `README.md` and inline docstrings.

---

## 5. File / Module Changes
- `scripts/mpc_optimizer.py` – rewritten (≈500 LOC).
- `scripts/mpc_optimizer_legacy.py` – archived.
- `tests/test_mpc_optimizer_equivalence.py` – new pytest file.
- `config/mpc_optimizer_config.yaml` – default parameter values, loaded into `PSDFOptimizerConfig`.
- Docs: update `PRD.md` references.

---

## 📋 Implementation Checklist
- [x] Backup existing `scripts/mpc_optimizer.py` → `scripts/mpc_optimizer_legacy.py`
- [x] Copy reference logic into new `scripts/mpc_optimizer.py`
- [x] Rename classes/functions (PSDFOptimizer ➔ MPCOptimizer)
- [x] Interface alignment with `psdf_ros_node.py`
- [x] YAML configuration loader (`PSDFOptimizerConfig`)
- [x] Unit tests for equivalence
- [ ] Integration test in simulation
- [ ] Documentation updates

---

## 6. Timeline & Milestones
| Day | Task |
| --- | --- |
| 0 | Approve plan |
| 1 | Backup & copy reference logic |
| 1 | Minimal rename + pass unit tests |
| 2 | Interface alignment with ROS node |
| 2 | Add configuration loading |
| 3 | Write tests & run in sim |
| 3 | Code cleanup, docs |

---

## 7. Risks & Mitigations
* **API mismatch** – Mitigation: add adapter layer and run static type checks.
* **Performance regression** – Benchmark solver time; enable warm-start.
* **Dependency issues** – Ensure versions in `requirements.txt` / `package.xml` include `acados_template`, `l4casadi`.

---

## 8. Acceptance Criteria
1. `mpc_optimizer.py` compiles & runs unit tests.
2. Solutions match `psdf_optimizer.py` within tolerance.
3. ROS node publishes planned trajectory without runtime errors.
4. Code coverage ≥ 80% for optimizer module.

---

## 9. Next Action
Wait for developer approval of this plan, then commence Step 1 (backup & copy reference logic).
