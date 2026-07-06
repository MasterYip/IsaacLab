# G1 Locomotion Tasks

## Environments

| Task ID | DOF | Robot Asset |
|---------|-----|-------------|
| `Isaac-Velocity-Rough-G1-v0` | 37 | `G1_MINIMAL_CFG` (USD, with hands) |
| `Isaac-Velocity-Rough-G1_MINIMAL_23DOF-v0` | 23 | `G1_MINIMAL_CFG` (USD, hands frozen) |
| `Isaac-Velocity-Rough-G1_29DOF-v0` | 29 | `G1_29DOF_CFG` (URDF, no hands) |
| `Isaac-Velocity-Flat-G1-v0` | 37 | `G1_MINIMAL_CFG` (USD, with hands) |

Play variants (smaller scene, no randomization): add `-Play` suffix, e.g. `Isaac-Velocity-Rough-G1-Play-v0`.

## Train

```bash
# 37 DOF (with hands) — rough terrain
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-G1-v0

# 29 DOF (no hands) — rough terrain
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-G1_29DOF-v0

# Minimal 29DOF (G1_MINIMAL_CFG, hands frozen) — rough terrain
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-G1_MINIMAL_23DOF-v0

# 37 DOF — flat terrain
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-G1-v0

# Common options
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-G1-v0 \
    --num_envs 4096 --max_iterations 3000 --seed 42
```

## Evaluate

```bash
# Play with a trained checkpoint (loads best model from logs by default)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-G1-v0

# 29 DOF variant
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-G1_29DOF-v0 --num_envs 48

# Minimal 29DOF variant (G1_MINIMAL_CFG, hands frozen)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-G1_MINIMAL_23DOF-v0 --num_envs 48

# Specify a checkpoint file directly
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-G1_29DOF-v0 \
    --checkpoint /path/to/model.pt

# Real-time playback with video recording
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-G1_29DOF-v0 \
    --real-time --video --video_length 500
```

## Export

Exported models (JIT `.pt` and ONNX `.onnx`) are automatically saved to `<log_dir>/exported/` after running `play.py`. You can also export manually:

```python
from isaaclab_rl.rsl_rl import export_policy_as_jit, export_policy_as_onnx

# After loading ppo_runner and policy:
export_policy_as_jit(ppo_runner.alg.policy, ppo_runner.obs_normalizer,
                     path="exported", filename="policy.pt")
export_policy_as_onnx(ppo_runner.alg.policy, normalizer=ppo_runner.obs_normalizer,
                      path="exported", filename="policy.onnx")
```

## Logs

Training logs are saved under `logs/rsl_rl/<experiment_name>/<timestamp>/`:
- `params/` — environment and agent config dumps
- `exported/` — exported JIT and ONNX models (after `play.py`)
- `videos/` — recorded videos (with `--video` flag)
