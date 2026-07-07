# 2026-07-07 — Waist DOF Settings Comparison

## Background

The 23-DOF G1 task (`G1_MINIMAL_CFG` with hands frozen → `torso_joint` only) trains fine.
The 29-DOF G1 task (3 waist joints: `waist_yaw`, `waist_roll`, `waist_pitch`) fails to train.

**Hypothesis:** The waist-related settings (Kp/Kd, reward weights, action scale, etc.) are the root cause.
This report compares our 29-DOF configs against the working reference from `unitree_rl_lab`.

## Reference

- **Our config (Unitree):** `config/g1/rough_env_cfg_unitree_29dof.py` — `UnitreeG129DOFRoughEnvCfg`
- **Our config (G1_29DOF):** `config/g1/rough_env_cfg_29dof.py` — `G129DOFRoughEnvCfg`
- **Reference (working):** `tmp/unitree_rl_lab/.../g1/29dof/velocity_env_cfg.py` — `RobotEnvCfg`

---

## 1. Actuator Kp/Kd (Robot Config)

Waist joints in `UNITREE_G1_29DOF_CFG` (shared by both our config and reference):

| Joint | Actuator Group | Kp | Kd | Effort | Velocity | Armature |
|-------|---------------|-----|-----|--------|----------|----------|
| `waist_yaw_joint` | N7520-14.3 | **200** | **5** | 88 | 32 | 0.01 |
| `waist_roll_joint` | N5020-16 | **40** | **5** | 25 | 37 | 0.01 |
| `waist_pitch_joint` | N5020-16 | **40** | **5** | 25 | 37 | 0.01 |

For comparison, the single `torso_joint` in `G1_MINIMAL_CFG` (23-DOF, works):
- Kp=**200**, Kd=**5**, effort=300, vel=100 (legs group)

Key observation: `waist_roll` and `waist_pitch` use the weak N5020-16 actuator (Kp=40) which is designed for arm joints — **only 1/5 the stiffness of `torso_joint`** (Kp=200) and only 25 N·m effort vs 300 N·m.

---

## 2. Reward Weights — Waist-Related

| Reward Term | Our Unitree | Our G1_29DOF | Reference | Notes |
|-------------|-------------|-------------|-----------|-------|
| `joint_deviation_waist` | **-0.1** | **-2.0** | **-1.0** | Reference uses `"waist.*"` regex |
| `joint_deviation_legs` | (in hip only) | (in hip only) | **-1.0** (hip_roll + hip_yaw) | Reference separates legs from hip |
| `flat_orientation_l2` | **-1.0** | **-1.0** | **-5.0** | Reference 5× heavier — critical for waist stability! |
| `action_rate_l2` | **-0.005** | **-0.005** | **-0.05** | Reference 10× heavier → smoother actions |
| `dof_pos_limits` | **-1.0** | **-1.0** | **-5.0** | Reference 5× heavier |
| `base_height_l2` | (not used) | (not used) | **-10.0** (target=0.78) | Keeps torso at correct height |
| `joint_vel_l2` | (not used) | (not used) | **-0.001** | Penalizes joint velocity |
| `feet_slide` | -0.1 | -0.1 | **-0.2** | Reference 2× heavier |

### Diagnostic signal

The reference penalizes `flat_orientation_l2` **5× more** than our configs. When waist joints are too weak (low Kp), the upper body tilts → robot falls. A heavier `flat_orientation_l2` reward would fight this, but it's a band-aid — the root cause is the Kp.

The reference also uses `base_height_l2` to keep the robot at 0.78 m — prevents waist collapse.

---

## 3. Action Scale

| Config | `joint_pos.scale` |
|--------|-------------------|
| Our Unitree | **0.5** (inherited from base) |
| Reference | **0.25** |

Reference uses **half** the action scale → smaller, more stable joint commands. With weaker waist Kp (40), large actions (0.5 scale) can over-command the waist, causing oscillation and instability.

---

## 4. Observations

| Aspect | Our Configs | Reference |
|--------|-------------|-----------|
| Policy `base_lin_vel` | **included** | **excluded** (only in critic) |
| Policy `base_ang_vel` scale | 1.0 (default) | **0.2** |
| Policy `joint_vel_rel` scale | 1.0 (default) | **0.05** |
| History length | 1 (default) | **5** |
| Privileged critic | **none** | **separate CriticCfg** with `base_lin_vel` |

Reference uses asymmetric actor-critic: policy sees less (no base_lin_vel), critic sees everything. This is standard for sim-to-real but less relevant for pure sim training stability.

---

## 5. Randomization & Events

| Setting | Our Configs | Reference |
|---------|-------------|-----------|
| `add_base_mass` | **None (disabled)** | **enabled** (-1.0, 3.0) on `torso_link` |
| `push_robot` | **None (disabled)** | **enabled** every 5s |
| `physics_material` friction_range | (0.8, 0.8) fixed | **(0.3, 1.0)** randomized |
| `reset_robot_joints.velocity_range` | **(0.0, 0.0)** | **(-1.0, 1.0)** |

Reference trains with **more** randomization, not less — this suggests the stability issue isn't from randomization but from the waist mechanics themselves.

---

## 6. Commands

| Setting | Our Configs | Reference |
|---------|-------------|-----------|
| `lin_vel_x` | **(0.0, 1.0)** | **(-0.5, 1.0)** |
| `lin_vel_y` | **(0.0, 0.0)** | **(-0.4, 0.4)** |
| `heading_command` | **True** | **False** |

---

## 7. Terminations

| Setting | Our Configs | Reference |
|---------|-------------|-----------|
| `base_contact` (illegal_contact on torso) | **enabled** | **not used** |
| `base_height` (root_height_below_minimum) | **not used** | **enabled** (min=0.2) |
| `bad_orientation` (limit_angle=0.8) | **not used** | **enabled** |

Reference uses orientation-based termination instead of contact-based.

---

## Summary: Root Cause

The primary issue is the **Kp/Kd mismatch for `waist_roll` and `waist_pitch`**:

| Joint | Current Kp | Should be closer to |
|-------|-----------|---------------------|
| `waist_yaw` | 200 | ✓ OK (matches torso_joint) |
| `waist_roll` | **40** | **≥ 200** |
| `waist_pitch` | **40** | **≥ 200** |

The N5020-16 actuator group puts `waist_roll` and `waist_pitch` at the same stiffness as shoulder/elbow joints (Kp=40). These joints support the entire upper body mass — they need leg-level stiffness (Kp ≥ 200), not arm-level.

Additionally, the reference compensates with:
1. **5× heavier `flat_orientation_l2`** reward (-5.0 vs -1.0)
2. **`base_height_l2`** reward to maintain torso height
3. **Half action scale** (0.25 vs 0.5) for smoother control
4. **`-5.0` dof_pos_limits** vs our -1.0

## Recommendations

1. **Move `waist_roll` and `waist_pitch` out of N5020-16** into N7520-14.3 (Kp=200, Kd=5) or their own group with Kp ≥ 200
2. Increase `flat_orientation_l2` weight to -5.0
3. Reduce action scale from 0.5 to 0.25
4. Add `base_height_l2` reward (weight=-10, target=0.78)
5. Consider increasing `dof_pos_limits` weight to -5.0

---

## Applied Changes (2026-07-07)

### 1. Robot Config — `UNITREE_G1_29DOF_CFG` (`unitree.py`)

N5020-16 actuator group — waist joints got dedicated Kp/Kd:

| Parameter | Before | After |
|-----------|--------|-------|
| `waist_roll` Kp | 40 | **80** |
| `waist_pitch` Kp | 40 | **80** |
| `waist_roll` Kd | 5 | **10** |
| `waist_pitch` Kd | 5 | **10** |

Other joints in N5020-16 (shoulders, elbow, wrist_roll, ankles) unchanged at Kp=40.

### 2. Env Config — Both 29DOF tasks (`rough_env_cfg_unitree_29dof.py` + `rough_env_cfg_29dof.py`)

| Setting | Before | After |
|---------|--------|-------|
| `flat_orientation_l2` weight | -1.0 | **-5.0** |
| `joint_deviation_waist` (combined) | -0.1 / -2.0 | *split* |
| `joint_deviation_waist_yaw` (new) | — | **-0.1** |
| `joint_deviation_waist_roll_pitch` (new) | — | **-2.0** / **-5.0** |

The combined waist deviation reward was split: `waist_yaw` gets a light penalty (-0.1) since it's needed for turning, while `waist_roll` and `waist_pitch` get a heavy penalty to keep them locked at 0° (they shouldn't move during locomotion).
