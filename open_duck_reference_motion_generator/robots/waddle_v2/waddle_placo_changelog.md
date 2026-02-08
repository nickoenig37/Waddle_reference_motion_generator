# `waddle_placo.urdf` — Changelog

## Overview

`waddle_placo.urdf` is a modified copy of `waddle.urdf` with joint origins, limits, and properties
adjusted to match the conventions used by `open_duck_mini_v2.urdf`. This is necessary because the
Placo walk engine expects specific joint zero configurations to generate sensible walking motions.

The **original `waddle.urdf`** was exported from OnShape with rotational offsets baked into several
joint origins (the RPY values). This means at joint angle = 0, the robot stands in a different pose
than the ODMv2 reference robot. Since Placo computes joint trajectories relative to the URDF zero
configuration, these offsets would produce incorrect motions.

**What's preserved:** All link masses, inertias, visual geometry, collision geometry, mesh references,
and the kinematic chain structure. The Waddle's heavier trunk (1.168 kg vs 0.699 kg due to Jetson
Nano + different battery) and heavier head (0.448 kg vs 0.353 kg) are untouched.

---

## Changes Made

### 1. Left Leg Joint Origins (RPY Offsets Removed)

| Joint | Original RPY | Fixed RPY | Why |
| ----- | ------------ | --------- | --- |
| `left_hip_pitch` | `-1.5708 0.785398 1.5708` | `3.14159 1.5708 0` | Had extra 45° offset. ODMv2 uses `(π, π/2, 0)` |
| `left_knee` | `0 0 1.5708` | `0 0 0` | Had 90° yaw offset. ODMv2 uses `(0, 0, 0)` |
| `left_ankle` | `0 0 -0.785398` | `0 0 0` | Had -45° yaw offset. ODMv2 uses `(0, 0, 0)` |

### 2. Right Leg Joint Origins (RPY Offsets Removed)

| Joint | Original RPY | Fixed RPY | Why |
| ----- | ------------ | --------- | --- |
| `right_hip_pitch` | `1.5708 0.785398 1.5708` | `0 1.5708 0` | Had extra 45° offset. ODMv2 uses `(0, π/2, 0)` |
| `right_knee` | `-3.14159 0 -1.5708` | `3.14159 0 0` | Had -90° yaw offset. ODMv2 uses `(π, 0, 0)` |
| `right_ankle` | `0 0 -0.785398` | `0 0 0` | Had -45° yaw offset. ODMv2 uses `(0, 0, 0)` |

### 3. Head/Neck Joint Origins (RPY Offsets Removed)

| Joint | Original RPY | Fixed RPY | Why |
| ----- | ------------ | --------- | --- |
| `neck_pitch` | `1.5708 -0.785398 0` | `1.5708 0 0` | Had -45° pitch offset. ODMv2 uses `(π/2, 0, 0)` |
| `head_pitch` | `0 0 -0.785398` | `0 0 0` | Had -45° yaw offset. ODMv2 uses `(0, 0, 0)` |

### 4. Joint Limits (Matched to ODMv2)

| Joint | Original Limits (rad) | Fixed Limits (rad) | Why |
| ----- | --------------------- | ------------------ | --- |
| `left_hip_pitch` | `[-0.436, 1.309]` | `[-1.222, 0.524]` | Limits must correspond to the new zero config |
| `left_knee` | `[-3.142, 0]` | `[-1.571, 1.571]` | Was negative-only due to offset; now symmetric |
| `left_ankle` | `[-0.785, 2.356]` | `[-1.571, 1.571]` | Was shifted due to offset; now symmetric |
| `right_hip_pitch` | `[-1.309, 0.436]` | `[-0.524, 1.222]` | Mirror of left, matches ODMv2 |
| `right_knee` | `[-3.142, 0]` | `[-1.571, 1.571]` | Was negative-only; now symmetric |
| `right_ankle` | `[-0.785, 2.356]` | `[-1.571, 1.571]` | Was shifted; now symmetric |
| `neck_pitch` | `[-1.134, 0.349]` | `[-0.349, 1.134]` | Limits were swapped due to offset |
| `head_pitch` | `[0, 1.571]` | `[-0.785, 0.785]` | Was positive-only; now centered |

### 5. Joint Effort and Velocity

| Property | Original | Fixed | Why |
| -------- | -------- | ----- | --- |
| `effort` | `10` (all joints) | `100` (all joints) | ODMv2 uses 100; higher values give Placo more headroom |
| `velocity` | `10` (all joints) | `100` (all joints) | Same reasoning |

### 6. Joint Friction Properties

Added `<joint_properties friction="0.0"/>` to all revolute joints to match ODMv2 format.

### 7. Minor RPY Cleanup

Normalized `-0` values to `0` in hip yaw and head joint RPY origins for consistency.

### 8. Removed MuJoCo-Specific Blocks

The following blocks were removed from the end of the file as they are not valid URDF
and are only used by MuJoCo:

- **`<sensor>` block** — gyro, accelerometer, velocimeter, frame sensors (MuJoCo-only XML)
- **`<default>` block** — STS3215 servo properties, backlash definitions (MuJoCo-only XML)

These remain in the original `waddle.urdf` for MuJoCo simulation use.

---

## What Was NOT Changed

| Property | Waddle Value | ODMv2 Value | Why Kept |
| -------- | ------------ | ----------- | -------- |
| `trunk_assembly` mass | 1.168 kg | 0.699 kg | Actual Waddle mass (Jetson Nano + battery) |
| `trunk_assembly` inertia | Larger | Smaller | Matches actual robot |
| `head_assembly` mass | 0.448 kg | 0.353 kg | Actual Waddle mass (cameras, speakers) |
| `trunk_assembly` COM | (-0.022, 0.001, 0.035) | (-0.048, 0, 0.039) | Actual Waddle COM position |
| `imu` frame + link | Present | Not present | Waddle has IMU mount point |
| Link/mesh geometry | Waddle meshes | ODMv2 meshes | Actual Waddle geometry |
| `left_hip_roll` RPY | Same | Same | Already matches ODMv2 |
| `left_hip_yaw` RPY | Same | Same | Already matches ODMv2 |
| `head_yaw` RPY | Same | Same | Already matches ODMv2 |
| `head_roll` RPY | Same | Same | Already matches ODMv2 |
| Foot frame origins | Same | Same | Already matches ODMv2 |
| Antenna joint origins | Same | Same | Already matches ODMv2 |

---

## Usage

Use `waddle_placo.urdf` with the reference motion generator:

```bash
# In WSL
UV_LINK_MODE=copy uv run open_duck_reference_motion_generator/gait_playground.py --duck waddle
```

Make sure the gait scripts reference `waddle_placo.urdf` as the model filename for the waddle robot.

---

## ⚠️ Important Caveats

1. **Visual geometry may appear wrong** — The mesh visual origins were designed for the original
   joint zero config. With the joint offsets removed, the meshes may render in incorrect positions
   at joint angle = 0. This is cosmetic only and does **not** affect the kinematics/dynamics that
   Placo uses for motion generation.

2. **These changes are for Placo only** — The original `waddle.urdf` should continue to be used
   for MuJoCo simulation (or re-exported to MuJoCo XML). The `waddle_placo.urdf` is specifically
   for the reference motion generator pipeline.

3. **Verify with gait_playground** — Always visually verify in the Meshcat viewer that the robot
   stands correctly before generating motions. If the initial pose looks wrong, the joint offsets
   may need further adjustment.
