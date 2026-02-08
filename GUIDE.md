# Waddle Reference Motion Generator — Guide

## Overview

This repository generates **reference walking motions** for duck-like bipedal robots using the [Placo](https://github.com/Rhoban/placo) walk engine. The output motions are used for **imitation learning** in RL training (e.g., in the Open Duck Playground).

---

## How It Works (Pipeline Overview)

1. **URDF robot model** → loaded by Placo (a walking motion planner)
2. **Placo Walk Engine** generates walking trajectories (joint angles over time)
3. **Polynomial fitting** compresses the trajectories into `polynomial_coefficients.pkl`
4. **Open Duck Playground** uses those coefficients as reference motions for RL rewards

---
## Main file you're going to run:
```bash
uv run open_duck_reference_motion_generator/gait_playground.py --duck waddle_v2
```

## Key Files

| File | Purpose |
| ---- | ------- |
| `open_duck_reference_motion_generator/gait_generator.py` | Core script that generates walking motions for a given robot |
| `open_duck_reference_motion_generator/gait_playground.py` | Interactive Flask web UI to tune gait parameters with Meshcat visualization |
| `open_duck_reference_motion_generator/placo_walk_engine.py` | Wraps Placo's walk engine with robot-specific configuration |
| `scripts/auto_waddle.py` | Batch generates motions (sweep or random) |
| `scripts/fit_poly.py` | Fits polynomial curves to generated motion data → outputs `polynomial_coefficients.pkl` |
| `scripts/replay_motion.py` | Replays a single generated motion for visualization |
| `scripts/replay_sweep.py` | Replays a sweep of generated motions |

---

## Robot Configuration

Each robot has a directory under `open_duck_reference_motion_generator/robots/` containing:

```text
robots/<robot_name>/
├── <robot_name>.urdf          # Robot description (used by Placo, NOT MuJoCo XML)
├── placo_defaults.json        # Default gait parameters (COM height, foot height, spacing, etc.)
├── auto_gait.json             # Ranges for sweep/random generation
├── placo_presets/             # Named parameter presets
│   ├── fast.json
│   ├── medium.json
│   └── slow.json
└── assets/                    # Mesh files referenced by the URDF
```

---

## Step-by-Step: Testing with an Existing Robot

> **⚠️ Windows Users: WSL Required**
>
> Placo and its dependencies (`cmeel-*`, `pinocchio`, `hpp-fcl`) are **Linux-only** packages that cannot be built on native Windows. You must run all `uv` commands through **WSL (Windows Subsystem for Linux)**.
>
> All commands below assume you are running inside WSL Ubuntu. Prefix commands with:
>
> ```bash
> wsl -d Ubuntu -- bash -lc "source ~/.local/bin/env; cd /mnt/c/<path_to_project>; <command>"
> ```
>
> Or simply open a WSL terminal directly.

### 1. Install `uv` (inside WSL)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.local/bin/env
```

### 2. Install dependencies

```bash
cd /mnt/c/Users/<you>/Desktop/.../Waddle_reference_motion_generator
UV_LINK_MODE=copy uv sync
```

### 2. Try the interactive playground (recommended first test)

```bash
uv run open_duck_reference_motion_generator/gait_playground.py --duck open_duck_mini_v2
```

This launches a **Flask web server** with a **Meshcat 3D viewer** where you can tune gait parameters (walk speed, foot height, COM height, etc.) and see the robot walk in real-time. This confirms Placo is working.

### 3. Generate a single motion

```bash
uv run open_duck_reference_motion_generator/gait_generator.py --duck open_duck_mini_v2 --preset fast --output_dir ./test_output -l 10
```

This generates a 10-second walking motion using the `fast` preset and saves it to `./test_output/`.

### 4. Generate a sweep of motions

```bash
uv run scripts/auto_waddle.py --duck open_duck_mini_v2 --num 10 --output_dir ./test_motions
```

### 5. Fit polynomials

```bash
uv run scripts/fit_poly.py -d ./test_motions
```

This produces `polynomial_coefficients.pkl`, which you'd copy to your playground's `data/` folder.

### 6. Replay a motion (optional verification)

```bash
uv run scripts/replay_motion.py -f ./test_output/<motion_file>.json
```

---

## Adapting for Your Waddle Robot

### Step 1: Create a Robot Directory

Create the following structure:

```text
open_duck_reference_motion_generator/robots/waddle/
├── waddle.urdf
├── placo_defaults.json
├── auto_gait.json
├── placo_presets/
│   ├── fast.json
│   └── medium.json
└── assets/
    └── (STL mesh files)
```

> **Tip:** Copy from `open_duck_mini_v2` as a starting point and adjust values.

### Step 2: Add `"waddle"` as a Duck Choice

In `gait_generator.py` (and `gait_playground.py`, `auto_waddle.py`), add `"waddle"` to the `--duck` argument choices:

```python
parser.add_argument(
    "--duck",
    choices=["go_bdx", "open_duck_mini", "open_duck_mini_v2", "waddle"],
    help="Duck type",
    required=True,
)
```

### Step 3: Tune `placo_defaults.json`

Key parameters to adjust for your Waddle's dimensions:

| Parameter | Description |
| --------- | ----------- |
| `walk_com_height` | Height of center of mass during walking |
| `feet_spacing` | Distance between the two feet |
| `foot_length` | Length of the foot |
| `walk_foot_height` | How high the foot lifts during a step |
| `walk_trunk_pitch` | Forward lean angle of the trunk |
| Joint names | Must match joint names in your URDF exactly |

### Step 4: Generate Motions and Fit Polynomials

```bash
# Generate motions
uv run scripts/auto_waddle.py --duck waddle --num 50 --output_dir ./waddle_motions

# Fit polynomial coefficients
uv run scripts/fit_poly.py -d ./waddle_motions
```

### Step 5: Copy Output to Playground

Copy the resulting `polynomial_coefficients.pkl` to:

```text
Open_Duck_Playground_Waddle/playground/waddle/data/polynomial_coefficients.pkl
```

---

## ⚠️ The URDF Problem

The reference motion generator requires a **URDF** file, not a MuJoCo XML. The Waddle robot currently only has MuJoCo XML descriptions (`waddle.xml`, `scene.xml`).

### Options to Get a URDF

1. **Export directly from OnShape** using `onshape-to-robot` with `"outputFormat": "urdf"` in your `config.json` (currently set to `"mujoco"`).

2. **Convert MuJoCo XML → URDF** manually or with conversion tools.

3. **Use `open_duck_mini_v2.urdf`** as a reference for joint naming conventions that Placo expects.

### Important Note on Servos

The Waddle uses **STS3215** servos (as defined in `joints_properties.xml`). Make sure the joint limits and effort values in your URDF match these servo specifications:

| Property | Value |
| -------- | ----- |
| Damping | `0.60` |
| Friction Loss | `0.052` |
| Armature | `0.028` |
| Force Range | `-3.35` to `3.35` N·m |
| Position Kp | `17.8` |
| Backlash | ±0.5° (`±0.00873 rad`) |

---

## Quick Reference: Full Workflow

```text
┌─────────────────────────────────────┐
│  1. Create URDF for Waddle robot    │
│     (from OnShape or convert XML)   │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  2. Set up robots/waddle/ directory │
│     (URDF, defaults, presets, mesh) │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  3. Test with gait_playground.py    │
│     (interactive tuning + Meshcat)  │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  4. Generate motions (auto_waddle)  │
│     (sweep across speed/direction)  │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  5. Fit polynomials (fit_poly.py)   │
│     → polynomial_coefficients.pkl   │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  6. Copy .pkl to Playground/data/   │
│     → used as RL reference motion   │
└─────────────────────────────────────┘
```
