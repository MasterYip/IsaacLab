# Elspider 4 Air Training

Problem:
- It is blocked by loading resources from the server.

Solution: Edit `source/extensions/omni.isaac.lab/omni/isaac/lab/utils/assets.py`:

```python
# NUCLEUS_ASSET_ROOT_DIR = carb.settings.get_settings().get("/persistent/isaac/asset_root/cloud")
# Use localhost server
NUCLEUS_ASSET_ROOT_DIR = "omniverse://localhost/NVIDIA/Assets/Isaac/4.2"
"""Path to the root directory on the Nucleus Server."""
```

Task names:
- Isaac-Velocity-Flat-ElSpider-Air-v0
- Isaac-Velocity-Rough-ElSpider-Air-v0

```bash
# conda
conda activate isaaclab
# Train
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py --task Isaac-Velocity-Flat-ElSpider-Air-v0  --num_envs 300 --max_iterations 6000 --resume true --headless
# Play
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Velocity-Flat-ElSpider-Air-v0  --num_envs 32
```
