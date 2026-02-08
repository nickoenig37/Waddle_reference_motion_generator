# URDF File Differences: `waddle.urdf` vs `open_duck_mini_v2.urdf`

This document details all identified differences between the Waddle robot URDF and the reference Open Duck Mini V2 URDF that could impact motion generation, simulation, and gait planning.

---

## PART 1: Structural & Kinematic Differences

### 1. Joint Origin RPY Offsets (45° / 90° Baked In) — 🔴 CRITICAL

The Onshape CAD has servo horns mounted at different orientations from the reference design. `onshape-to-robot` has baked these rotations into the joint origins, shifting the zero-position of every affected joint.

| Joint | `waddle` rpy | `open_duck_mini_v2` rpy | Offset |
|---|---|---|---|
| `left_hip_pitch` | `-1.5708 0.785398 1.5708` | `3.14159 1.5708 0` | **45° offset** |
| `right_hip_pitch` | `1.5708 0.785398 1.5708` | `0 1.5708 0` | **45° offset** |
| `left_knee` | `0 0 1.5708` | `0 0 0` | **90° in z** |
| `right_knee` | `-3.14159 0 -1.5708` | `-3.14159 0 0` | **90° in z** |
| `left_ankle` | `0 0 -0.785398` | `0 0 0` | **-45° in z** |
| `right_ankle` | `0 0 -0.785398` | `0 0 0` | **-45° in z** |
| `neck_pitch` | `1.5708 -0.785398 0` | `1.5708 0 0` | **-45° in pitch** |
| `head_pitch` | `0 0 -0.785398` | `0 0 0` | **-45° in z** |

**Impact:** When joint angle = 0, the robot's physical pose is rotated 45° (or 90°) from the reference at each affected joint. Any reference motions, IK solutions, or Placo-generated trajectories designed for `open_duck_mini_v2` will produce **wrong physical poses** on the Waddle robot.

**Recommended Fix:** Fix the servo horn orientations in the Onshape assembly so the zero positions align with the reference, then re-export with `onshape-to-robot`.

---

### 2. Joint Limits (Shifted to Compensate for RPY Offsets) — 🔴 CRITICAL

The joint limits have been shifted to match the offset zero positions, but they no longer match the reference:

| Joint | `open_duck_mini_v2` | `waddle` | Status |
|---|---|---|---|
| `left_hip_yaw` | [-0.5236, 0.5236] | [-0.523599, 0.523599] | ✅ Match |
| `right_hip_yaw` | [-0.5236, 0.5236] | [-0.523599, 0.523599] | ✅ Match |
| `left_hip_roll` | [-0.4363, 0.4363] | [-0.436332, 0.436332] | ✅ Match |
| `right_hip_roll` | [-0.4363, 0.4363] | [-0.436332, 0.436332] | ✅ Match |
| `left_hip_pitch` | [-1.22, 0.52] | [-0.436332, 1.309] | ❌ Shifted ~0.785 |
| `right_hip_pitch` | [-0.52, 1.22] | [-1.309, 0.436332] | ❌ Shifted ~0.785 |
| `left_knee` | [-1.5708, 1.5708] | [-3.14159, 0] | ❌ Shifted ~1.5708 |
| `right_knee` | [-1.5708, 1.5708] | [-3.14159, 0] | ❌ Shifted ~1.5708 |
| `left_ankle` | [-1.5708, 1.5708] | [-0.785398, 2.35619] | ❌ Shifted ~0.785 |
| `right_ankle` | [-1.5708, 1.5708] | [-0.785398, 2.35619] | ❌ Shifted ~0.785 |
| `neck_pitch` | [-0.3491, 1.1345] | [-1.13446, 0.349066] | ❌ Inverted |
| `head_pitch` | [-0.7854, 0.7854] | [0, 1.5708] | ❌ Shifted ~0.785 |
| `head_yaw` | [-2.7925, 2.7925] | [-2.79253, 2.79253] | ✅ Match |
| `head_roll` | [-0.5236, 0.5236] | [-0.523599, 0.523599] | ✅ Match |
| `left_antenna` | [-1.5708, 1.5708] | [-1.5708, 1.5708] | ✅ Match |
| `right_antenna` | [-1.5708, 1.5708] | [-1.5708, 1.5708] | ✅ Match |

**Impact:** Placo uses joint limits to compute feasible trajectories. Shifted/inverted limits cause the motion generator to bend joints in wrong directions or exceed physical limits.

**Recommended Fix:** Will auto-fix when RPY offsets (issue #1) are corrected.

---

### 3. Hip Yaw Mounting Position & Orientation — 🔴 CRITICAL

The hip yaw motor is mounted at a different position on the trunk and with a different frame orientation.

**Left Hip Yaw:**
```xml
<!-- waddle -->
<origin xyz="-0.019 0.035 0.0459409" rpy="0 0 0"/>

<!-- open_duck_mini_v2 -->
<origin xyz="-0.0445 0.035 0.0553909" rpy="-1.5708 0 3.14159"/>
```

**Right Hip Yaw:**
```xml
<!-- waddle -->
<origin xyz="-0.019 -0.035 0.0459409" rpy="0 0 0"/>

<!-- open_duck_mini_v2 -->
<origin xyz="-0.0445 -0.035 0.0553909" rpy="-1.5708 0 3.14159"/>
```

| Axis | `waddle` | `open_duck_mini_v2` | Delta |
|---|---|---|---|
| x | -0.019 | -0.0445 | **25.5 mm difference** |
| y | ±0.035 | ±0.035 | ✅ Match |
| z | 0.0459 | 0.0554 | **9.5 mm difference** |
| rpy | `0 0 0` | `-1.5708 0 3.14159` | **Completely different frame** |

**Impact:** The legs attach at a different point on the trunk AND the yaw joint frame is oriented differently. The yaw rotation axis operates in a different plane relative to the trunk. All IK solutions and walking gaits will produce different foot placements.

**Recommended Fix:** Verify the CAD placement of the hip yaw motor matches the reference design. The position difference may be intentional (different trunk geometry), but the frame orientation difference needs investigation.

---

### 4. Trunk Inertial Properties — 🟠 MODERATE

| Property | `waddle` | `open_duck_mini_v2` | Delta |
|---|---|---|---|
| mass | 1.1678 kg | 0.83978 kg | **+39%** |
| CoM x | -0.02151 | -0.02381 | +2.3 mm |
| CoM y | 0.00119 | 0.000534 | +0.66 mm |
| CoM z | 0.03540 | 0.02866 | **+6.7 mm higher** |
| ixx | 0.002527 | 0.001061 | **2.4×** |
| iyy | 0.005655 | 0.002345 | **2.4×** |
| izz | 0.004820 | 0.001955 | **2.5×** |

**Impact:** The heavier trunk with a higher center of mass significantly affects balance dynamics, walking gait stability, and sim-to-real transfer.

**Recommended Fix:** Expected difference due to different components (Jetson Nano, different battery, etc.). Gait parameters will need tuning for the Waddle robot's mass distribution.

---

### 5. Matching Elements ✅

The following elements match between the two URDFs:

- `left_hip_roll` origin: ✅ Position and RPY match
- `right_hip_roll` origin: ✅ Match
- `head_yaw` joint: ✅ Match
- `head_roll` joint: ✅ Match
- `left/right_antenna` joints: ✅ Match
- `trunk_frame` fixed joint: ✅ Match
- `imu_frame` fixed joint: ✅ Match
- `left_foot_frame` / `right_foot_frame`: ✅ Match
- `head_frame`: ✅ Match
- Sensor configuration: ✅ Match
- Joint properties (default classes): ✅ Populated correctly

---

## PART 2: Mesh & Collision Differences

### Commented-Out Part in Reference

The reference `open_duck_mini_v2.urdf` has **one commented-out visual/collision pair** in `trunk_assembly`:

| Part | Mesh File | Status in Reference | Status in Waddle |
|---|---|---|---|
| `wj_wk00_0123middlecase_56` (neck pitch servo body) | `wj_wk00_0123middlecase_56.stl` | **Commented out** | **Active (included)** |

Location in reference (commented out):
```xml
<!-- <visual>
    <origin xyz="0.001 0.0097 0.0645..." rpy="0 1.5708 0"/>
    <geometry>
        <mesh filename="package://assets/wj_wk00_0123middlecase_56.stl"/>
    </geometry>
</visual>
<collision>
    ...same origin...
    <geometry>
        <mesh filename="package://assets/wj_wk00_0123middlecase_56.stl"/>
    </geometry>
</collision> -->
```

The Waddle URDF has this part **active** in `trunk_assembly` at origin `xyz="0.001 0.0097 0.0645009" rpy="0 1.5708 0"`.

**Impact:** Minor — the reference likely commented it out to avoid collision overlap with the same servo appearing in `neck_pitch_assembly`. If collision is enabled on your trunk, this could cause self-collision interference.

---

### Missing Collision Geometry in Waddle URDF

The Waddle URDF only has collision geometry on **2 elements** (feet). The reference has collision geometry on virtually every visual element.

#### `trunk_assembly` — Missing Collisions

| Part | Mesh File | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|---|
| `jetson_nano_baseplate` | `jetson_nano_baseplate.stl` | N/A (Waddle-only part) | ❌ |
| `body_front` | `body_front.stl` | ✅ | ❌ |
| `battery_enclosure` | `battery_enclosure.stl` | N/A (Waddle-only part) | ❌ |
| `turnigy_3s_battery` | `turnigy_3s_battery.stl` | N/A (Waddle-only part) | ❌ |
| `body_middle_bottom` | `body_middle_bottom.stl` | ✅ | ❌ |
| `simplified_jetson_nano` | `simplified_jetson_nano.stl` | N/A (Waddle-only part) | ❌ |
| `switch` | `switch.stl` | N/A (Waddle-only part) | ❌ |
| `body_middle_top` | `body_middle_top.stl` | ✅ | ❌ |
| `body_back` | `body_back.stl` | ✅ | ❌ |
| `bno055` | `bno055.stl` | ✅ | ❌ |
| `roll_bearing` (×2) | `roll_bearing.stl` | ✅ | ❌ |
| `trunk_bottom` | `trunk_bottom.stl` | ✅ | ❌ |
| `trunk_top` | `trunk_top.stl` | ✅ | ❌ |
| `board` | `board.stl` | N/A (Waddle-only part) | ❌ |
| `pdb_xt60` | `pdb_xt60.stl` | N/A (Waddle-only part) | ❌ |
| All servo parts (×10+) | `wj_wk00_*.stl`, `drive_palonier.stl`, `passive_palonier.stl` | ✅ | ❌ |

#### `hip_roll_assembly` / `hip_roll_assembly_2` — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `roll_motor_bottom` | ✅ | ❌ |
| `roll_motor_top` | ✅ | ❌ |
| All servo parts (×5) | ✅ | ❌ |

#### `left_roll_to_pitch_assembly` / `right_roll_to_pitch_assembly` — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `left_roll_to_pitch` / `right_roll_to_pitch` | ✅ | ❌ |
| All servo parts (×5) | ✅ | ❌ |

#### `knee_and_ankle_assembly` (×4 links) — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `left_cache` / `right_cache` | ✅ | ❌ |
| `leg_spacer` | ✅ | ❌ |
| `left/right_knee_to_ankle_left/right_sheet` | ✅ | ❌ |
| All servo parts (×5) | ✅ | ❌ |

#### `foot_assembly` / `foot_assembly_2` — Partial Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `foot_side` | ✅ | ❌ |
| `foot_bottom_tpu` | ✅ | ✅ |
| `foot_bottom_pla` | ✅ | ❌ |
| `foot_top` | ✅ | ❌ |

#### `neck_pitch_assembly` — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `neck_left_sheet` | ✅ | ❌ |
| `neck_right_sheet` | ✅ | ❌ |
| All servo parts (×5) | ✅ | ❌ |

#### `head_pitch_to_yaw` — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `head_pitch_to_yaw` | ✅ | ❌ |

#### `neck_yaw_assembly` — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `head_yaw_to_roll` | ✅ | ❌ |
| All servo parts (×5) | ✅ | ❌ |

#### `head_assembly` — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `usb_camera_ov2710` (×2) | ✅ | ❌ |
| `head_bot_sheet` | ✅ | ❌ |
| `head` | ✅ | ❌ |
| `flash_reflector_interface` | ✅ | ❌ |
| `bulb` (×2) | ✅ | ❌ |
| `glass` (×4) | ✅ | ❌ |
| `head_roll_mount` | ✅ | ❌ |
| `left_eye` / `right_eye` | ✅ | ❌ |
| `flash_light_module` | ✅ | ❌ |
| `full_speaker` | ✅ | ❌ |
| `roll_bearing` | ✅ | ❌ |
| `sg90` (×2) | ✅ | ❌ |
| `speaker_interface` / `speaker_stand` | ✅ | ❌ |
| All servo parts (×5) | ✅ | ❌ |

#### `left_antenna_holder` / `right_antenna_holder` — Missing Collisions

| Part | Has Collision in Reference | Has Collision in Waddle |
|---|---|---|
| `left_antenna_holder` / `right_antenna_holder` | ✅ | ❌ |

---

### Parts Present in Waddle but NOT in Reference

| Part | Location | Notes |
|---|---|---|
| `jetson_nano_baseplate` | `trunk_assembly` | Custom addition (Jetson Nano mount) |
| `simplified_jetson_nano` | `trunk_assembly` | Custom addition (compute module) |
| `turnigy_3s_battery` | `trunk_assembly` | Different battery model |
| `battery_enclosure` | `trunk_assembly` | Custom addition (battery holder) |
| `switch` | `trunk_assembly` | Custom addition (power switch) |
| `pdb_xt60` | `trunk_assembly` | Custom addition (power distribution) |
| `board` | `trunk_assembly` | Custom addition (circuit board) |

### Parts Present in Reference but NOT in Waddle

| Part | Location | Notes |
|---|---|---|
| `esp32` | `trunk_assembly` | Reference main controller |
| `feetech_board` | `trunk_assembly` | Reference servo driver board |
| `battery` (original) | `trunk_assembly` | Reference battery model |

---

## PART 3: Priority Fix Summary

| Priority | Issue | Impact | Recommended Fix |
|---|---|---|---|
| 🔴 **P0** | 45°/90° RPY offsets at 8 joints | All reference motions produce wrong poses | Fix servo horn orientation in Onshape CAD |
| 🔴 **P0** | Joint limits shifted/inverted at 8 joints | Placo generates incorrect trajectories | Auto-fixes when RPY offsets are fixed |
| 🔴 **P0** | Hip yaw position (25.5mm x, 9.5mm z) AND frame orientation different | Entire leg kinematic chain attached differently | Verify CAD matches reference hip yaw placement |
| 🟠 **P1** | Missing collision geometry on all links except feet | No body/leg collisions in simulation | Add `<collision>` tags mirroring `<visual>` tags |
| 🟡 **P2** | Trunk mass 39% heavier (1.168 vs 0.840 kg) | Balance and gait dynamics differ | Expected — tune gait parameters |
| 🟡 **P2** | Trunk CoM shifted ~6.7mm higher | Affects balance point | Expected from different components |
| 🟢 **P3** | Commented-out neck servo in reference vs active in Waddle | Potential self-collision overlap | Consider commenting out to match reference |
| 🟢 **P3** | Extra parts (Jetson, battery, PDB, switch) | Contributes to mass/inertia differences | Expected design differences |