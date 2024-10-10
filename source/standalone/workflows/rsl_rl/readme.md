# Elspider 4 Air Training

Task names:
- Isaac-Velocity-Flat-ElSpider-Air-v0
- Isaac-Velocity-Rough-ElSpider-Air-v0

```bash
# conda
conda activate isaaclab
# Train
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py --task Isaac-Velocity-Flat-ElSpider-Air-v0  --num_envs 300 --max_iterations 6000 --resume true
# Play
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Velocity-Flat-ElSpider-Air-v0  --num_envs 32
```
