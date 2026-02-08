# Generating the Polynomial Coefficients Reference Guide

## First off, what is this file?
It's a compact mathematical representation of walking motions. Instead of storing thousands of timestamped joint angle values, it stores polynomial curves that can recreate any walking pattern on demand.

### How It Works:
Walking Motion (raw)          Polynomial Fit              Usage in MuJoCo
─────────────────────         ──────────────              ───────────────
                              
t=0.0: [0.1, -0.3, ...]      For each joint:             Given (dx, dy, dtheta, phase):
t=0.01: [0.1, -0.29, ...]    q(phase) = a₀ + a₁p +      → evaluate polynomials
t=0.02: [0.11, -0.28, ...]     a₂p² + a₃p³ + ...        → get target joint angles
...                                                       → compute RL reward:
t=0.5: [0.3, 0.1, ...]       Coefficients indexed by       reward = -||q_actual - q_ref||²
...                           (dx, dy, dtheta)
t=1.0: [0.1, -0.3, ...]      
(one full gait cycle)         


How MuJoCo RL Training Uses It
During reinforcement learning in the Open Duck Playground:

Policy outputs joint angle commands → robot moves in MuJoCo
Reference motion reward: At each timestep, the trainer:
Takes the current commanded velocity (dx, dy, dtheta)
Finds the nearest polynomial entry (or interpolates)
Evaluates q_ref = polynomial(current_phase) for each joint
Computes reward += -weight * ||q_actual - q_ref||²
This guides the RL policy to discover gaits similar to the Placo-planned walk, while still allowing the policy freedom to adapt


## Okay now running things:
### Step 1: Visual Verification with Gait Playground
- First make sure you've properly set your robot up to work in the Placo Gait Generator

You can verify this by running this from your Waddle_reference_motion_generation spot:
```bash
UV_LINK_MODE=copy uv run open_duck_reference_motion_generator/gait_playground.py --duck waddle_v2
``` 

### Step 2: Generate a Single Test Motion Test with one motion first to catch errors quickly:

```bash
UV_LINK_MODE=copy uv run open_duck_reference_motion_generator/gait_generator.py \
    --duck waddle_v2 \
    -n waddle_v2_test_walk \
    --dx 0.04 \
    --dy 0.0 \
    --dtheta 0.0 \
    -o ./waddle_v2_test \
    -l 8
```
This generates an 8-second forward walk at 0.04 m/s. Check the output JSON:
```bash
ls ./waddle_v2_test/
```

### Step 3: Replay and Verify the Test Motion
```bash
UV_LINK_MODE=copy uv run scripts/replay_motion.py \
    -f ./waddle_v2_test/<filename>.json
```
This should show the robot walking in the Meshcat viewer. Verify it looks correct before proceeding.

### Step 4: Generate the Full Sweep of Motions
This is the big one — generates motions across all velocity combinations defined in your auto_gait.json:
**First**: Ensure you have at least a `medium.json` preset file in:
```
open_duck_reference_motion_generator/robots/waddle_v2/placo_presets/medium.json
```

This file contains the gait parameters for the sweep. See the example files in `open_duck_mini_v2/placo_presets/` for reference.

Then generate the sweep:

```bash
UV_LINK_MODE=copy uv run scripts/auto_waddle.py \
    --duck waddle_v2 \
    --sweep \
    --output_dir ./waddle_v2_motions
```

This will generate ~350 motions (the exact number depends on your `auto_gait.json` sweep parameters).
**Estimated time**: 15-45 minutes depending on CPU.


Optional flags:
- `-j N` — number of parallel jobs (speeds up generation)
- `-v` — verbose output
- `--plot` — plot the generated motions
- `--num N` — number of random motions (used without `--sweep`)
// ...existing code...


### Step 5: Fit Polynomials
```bash
UV_LINK_MODE=copy uv run scripts/fit_poly.py \
    -d ./waddle_v2_motions
```
