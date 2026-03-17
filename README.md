# 🚬 Inverse Kinematics Arm Controller

A **Godot 4** project demonstrating a custom **Cyclic Coordinate Descent (CCD)** Inverse Kinematics solver for a 4-DOF robotic arm holding a cigarette.

![Godot](https://img.shields.io/badge/Godot-4.x-blue?logo=godot-engine)
![License](https://img.shields.io/badge/license-MIT-green)

---

## 🔎 Demo 

<table width="100%">
  <tr>
    <td width="50%" align="center" valign="top">
      <img alt="CCD Visualization" src="https://github.com/user-attachments/assets/ef29899a-3f87-4fce-80dd-88004538da91" width="360">
    </td>
    <td width="50%" align="center" valign="top">
      <img alt="Real time demo" src="https://github.com/user-attachments/assets/e0a2c688-99c5-4b7f-bb76-f10369f60567" width="360">
    </td>
  </tr>
  <tr>
    <td width="50%" align="left" valign="top">
      Visualization of the Cyclic Coordinate Descent algorithm. The blue, red, and green vectors are the joint, current and target vectors respectively.
    </td>
    <td width="50%" align="left" valign="top">
      The algorithm working in real time.
    </td>
  </tr>
</table>


## 🚀 Quick Start

### Option 1: Download Pre-built Binaries (Recommended)

Download the latest release for your platform:

| Platform | Download |
|----------|----------|
| 🐧 Linux | [IKArmDemo.x86_64](https://github.com/DanielHidalgoChica/smoker-IK-arm-3D/releases/download/1.0.0/smokerIKArm_Linuz_v1.0.0.zip) |
| 🪟 Windows | [IKArmDemo.exe](https://github.com/DanielHidalgoChica/smoker-IK-arm-3D/releases/download/1.0.0/smokerIKArm_Windows_v1.0.0.zip) |


### Option 2: Import into Godot Editor

1. Install [Godot 4.x](https://godotengine.org/download)
2. Clone this repository:
   ```git clone https://github.com/DanielHidalgoChica/smoker-IK-arm-3D.git```
3. Import project on Godot
4. Press **F5** to run


## 🎯 Project Overview

This project implements a real-time IK system that allows a character's arm to reach target positions in 3D space. The arm can bring a cigarette to the character's mouth or extend it toward clicked positions in the scene.


## 🧮 The Algorithm: Cyclic Coordinate Descent

### Overview

CCD is an iterative IK algorithm that adjusts one joint at a time, cycling through the kinematic chain from the **end-effector toward the root**. Each iteration reduces the distance between the end-effector and the goal.

![CCD Geometric Diagram](https://github.com/user-attachments/assets/442f4e7a-47ec-4e58-87fc-299306c0aebd)
> *CCD iteration in 2D (Source: [Ryan Juckett](https://www.ryanjuckett.com/cyclic-coordinate-descent-in-2d/))*

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

$$\mathbf{v}_{cur}^{p} = \mathbf{v}_{cur} - (\mathbf{v}_{cur} \cdot \hat{\mathbf{a}}) \hat{\mathbf{a}}$$

$$\mathbf{v}_{tgt}^{p} = \mathbf{v}_{tgt} - (\mathbf{v}_{tgt} \cdot \hat{\mathbf{a}}) \hat{\mathbf{a}}$$

#### Step 3: Compute the Signed Angle
Assuming every vector is normalized (we do it) the signed correction angle for the joint can be easily computed:

$$\sin(\theta) = \hat{\mathbf{a}} \cdot (\hat{\mathbf{v}}_{cur}^{p} \times \hat{\mathbf{v}}_{tgt}^{p})$$

$$\cos(\theta) = \hat{\mathbf{v}}_{cur}^{p} \cdot \hat{\mathbf{v}}_{tgt}^{p}$$

$$\theta = \arctan(\sin(\theta), \cos(\theta))$$

#### Step 4: Apply with Damping and Limits

The correction is clamped for stability and joint constraints:

```
δ = clamp(θ, -step_max, +step_max)
new_angle = clamp(current_angle + δ, joint_min, joint_max)
```

---

## 🦾 Kinematic Chain

The arm consists of 4 joints with different rotation axes:

| Joint | Node | Rotation Axis | Range |
|-------|------|---------------|-------|
| Shoulder | `shoulder_pivot` | X | -180° to 0° |
| Elbow | `elbow_pivot` | Z | 0° to 90° |
| Wrist Roll | `wrist_roll_pivot` | Y (roll) | -130° to -40° |
| Wrist Pitch | `wrist_pitch_pivot` | Z (pitch) | -60° to 60° |



## 🎮 Forward Kinematics Controls

| Action | Input |
| :--- | :--- |
| **Shoulder Joint** | `1`, `2` keys |
| **Elbow Joint** | `3`, `4` keys |
| **Wrist Pitch** | `5`, `6` keys |
| **Wrist Roll** | `7`, `8` keys |


## ⚙️ Inverse Kinematics (IK) Controls

| Action | Input |
| :--- | :--- |
| **Step effector towards mouth** | `Space` |
| **Move effector smoothly to mouth** | `Enter` |
| **Move effector to green table** | `Left Click` (on table) |
| **Snap effector to cigarette tip** | `F` key |
| **Snap effector to cigarette filter** | `M` key |

To use FK controls after an IK action, one must press `Space` at least one to snap out of the "solving" mode.

---

## 🔧 Implementation Details

### Forward Kinematics (FK) Cache

For efficiency, the solver uses a **cached FK computation** to evaluate end-effector positions without modifying or reading from the scene:

```gdscript
func get_cached_transform3d(output: Array[float], upto_idx: int) -> Transform3D
```



## 🛠️ Technologies

- **Godot 4.x** - Game engine
- **GDScript** - Scripting language
- **Debug Draw 3D** - Visualization addon for debugging

---

## Acknowledgments
The core concept for this project was inspired by the 2D Inverse Kinematics implementation found in [Gab-ani/ik-experiments](https://github.com/Gab-ani/ik-experiments). This repository adapts and extends those foundational ideas into a 3D environment.

## 👤 Author

**Daniel Hidalgo Chica**  
Universidad de Granada


