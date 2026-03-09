# 🚬 Inverse Kinematics Arm Controller

A **Godot 4** project demonstrating a custom **Cyclic Coordinate Descent (CCD)** Inverse Kinematics solver for a 4-DOF robotic arm holding a cigarette.

![Godot](https://img.shields.io/badge/Godot-4.x-blue?logo=godot-engine)
![License](https://img.shields.io/badge/license-MIT-green)

## 🎯 Project Overview

This project implements a real-time IK system that allows a character's arm to reach target positions in 3D space. The arm can bring a cigarette to the character's mouth or extend it toward clicked positions in the scene.

### Features

- **Custom CCD IK Solver** - Lightweight, iterative algorithm with joint limits
- **Smooth Animation** - Interpolated movement using `lerp_angle()` for natural motion
- **Dual End-Effectors** - Switch between cigarette tip and mouth tip
- **Interactive Control** - Click anywhere to set targets via raycasting
- **Debug Visualization** - Real-time display of rotation axes and correction vectors

---

## 🧮 The Algorithm: Cyclic Coordinate Descent

### Overview

CCD is an iterative IK algorithm that adjusts one joint at a time, cycling through the kinematic chain from the **end-effector toward the root**. Each iteration reduces the distance between the end-effector and the goal.

### The Geometric Core

The key insight of this implementation lies in how we compute the rotation correction for each joint:

#### Step 1: Define the Vectors

For each joint $i$, we compute two vectors originating from the joint position $\mathbf{J}_i$:

- **Current vector**: $\mathbf{v}_{cur} = \mathbf{E} - \mathbf{J}_i$ (joint → end-effector)
- **Target vector**: $\mathbf{v}_{tgt} = \mathbf{G} - \mathbf{J}_i$ (joint → goal)

Where $\mathbf{E}$ is the end-effector position and $\mathbf{G}$ is the goal position.

#### Step 2: Project onto the Rotation Plane

Each joint rotates around a single axis $\hat{\mathbf{a}}$ (e.g., X, Y, or Z in local space). The rotation only affects components **perpendicular** to this axis.

We project both vectors onto the plane perpendicular to $\hat{\mathbf{a}}$:

$$\mathbf{v}_{cur}^{\perp} = \mathbf{v}_{cur} - (\mathbf{v}_{cur} \cdot \hat{\mathbf{a}}) \hat{\mathbf{a}}$$

$$\mathbf{v}_{tgt}^{\perp} = \mathbf{v}_{tgt} - (\mathbf{v}_{tgt} \cdot \hat{\mathbf{a}}) \hat{\mathbf{a}}$$

#### Step 3: Compute the Signed Angle

The rotation angle $\theta$ between the projections is computed using the **atan2** function for proper sign handling:

$$\sin(\theta) = \hat{\mathbf{a}} \cdot (\hat{\mathbf{v}}_{cur}^{\perp} \times \hat{\mathbf{v}}_{tgt}^{\perp})$$

$$\cos(\theta) = \hat{\mathbf{v}}_{cur}^{\perp} \cdot \hat{\mathbf{v}}_{tgt}^{\perp}$$

$$\theta = \text{atan2}(\sin(\theta), \cos(\theta))$$

#### Step 4: Apply with Damping and Limits

The correction is clamped for stability and joint constraints:

```
δ = clamp(θ, -step_max, +step_max)
new_angle = clamp(current_angle + δ, joint_min, joint_max)
```

## 🦾 Kinematic Chain

The arm consists of 4 joints with different rotation axes:

| Joint | Node | Rotation Axis | Range |
|-------|------|---------------|-------|
| Shoulder | `shoulder_pivot` | X (pitch) | -180° to 0° |
| Elbow | `elbow_pivot` | Z (flexion) | 0° to 90° |
| Wrist Roll | `wrist_roll_pivot` | Y (roll) | -130° to -40° |
| Wrist Pitch | `wrist_pitch_pivot` | Z (pitch) | -60° to 60° |

---

## 🎮 Controls

| Input | Action |
|-------|--------|
| **Left Click** | Set target position (raycast) |
| `ik_solve` | Solve IK and animate to mouth |
| `ik_step` | Single CCD iteration (debug) |
| `tip_mouth` | Use mouth tip as end-effector |
| `tip_fire` | Use fire tip as end-effector |
| `return-default` | Return to rest pose |

---

## 🔧 Implementation Details

### Forward Kinematics (FK) Cache

The solver uses a **cached FK computation** to evaluate end-effector positions without modifying or reading from the scene:

```gdscript
func get_cached_transform3d(output: Array[float], upto_idx: int) -> Transform3D
```

This allows mathematically testing hypothetical joint configurations before applying them.

### Smooth Animation

Instead of snapping to solutions, the solver stores target angles and interpolates in `_process()`:

```gdscript
current_euler.x = lerp_angle(current_euler.x, goal, smoothing_speed * delta)
```

---

## 🛠️ Technologies

- **Godot 4.x** - Game engine
- **GDScript** - Scripting language
- **Debug Draw 3D** - Visualization addon for debugging

---

## 👤 Author

**Daniel Hidalgo Chica**  
Universidad de Granada

---
