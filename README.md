# Self-Driving Car Simulation (Unity)

An autonomous robot car in Unity that drives itself around a track and avoids obstacles. It has no hard-coded route: every decision comes from raycast sensors, an orientation sensor, and physics-based wheel control. That means it works on track layouts it has never seen.

Built as part of the MSc Artificial Intelligence and Robotics at the University of Hertfordshire (Artificial Life module).

---

## Overview

The robot is controlled by a single script, `RobotController.cs`. Each physics step it:

1. **Senses** the track edges, nearby obstacles, its speed, the road slope and its own rotation.
2. **Decides** how to steer and how much power to apply, using a priority-based control loop.
3. **Acts** by driving four Unity `WheelCollider`s and updating the visible wheel meshes.

The robot was scored on how far it gets around the course, how long it takes, how smooth its path is, and whether it avoids collisions with the track sides and obstacles.

---

## Features

| Capability | How it works |
|---|---|
| **Track following** | Left and right raycast sensors point 45° outwards and tilt down towards the road. If one side loses the road, the car steers back. If both sides see road, it balances the two distances to stay centred, with a small dead zone to stop jitter. |
| **Obstacle avoidance** | The front sensor sends a fan of 5 rays across ±20°. Each ray that hits something pushes the steering away from its side, weighted by how close the obstacle is. The car also slows down in proportion to the closest hit. |
| **Priority control** | Obstacle avoidance overrides track following. An avoidance cooldown (1 s) stops the car swerving straight back towards an obstacle it has just passed. |
| **Speed control** | Holds a target speed using the Rigidbody velocity. Power is shared between the wheels: 100% to the rear and 70% to the front. |
| **Slope compensation** | A downward ray measures the ground normal and works out the forward slope, so the car adds power uphill and holds back downhill. |
| **Yaw-rate stabilisation** | Uses the change in yaw angle to estimate how fast the car is turning, then applies gentle counter-steer to damp oscillation and keep the path smooth. |
| **Anti-roll bar** | Compares suspension compression on the left and right wheels and applies opposing forces to reduce body roll on corners and banked surfaces. Includes guards against divide-by-zero and NaN. |
| **Self-righting** | If side tilt goes above 35°, the car is smoothly rotated back upright to stop it flipping. |
| **Grip tuning** | Raises forward and sideways tyre friction stiffness for better traction. |

---

## Sensors and actuators

**Sensors** (all rotations are set in code; positions and scale are not changed)
- `RaycastSensorFront`: obstacle detection. Pitched 3° down and sends a 5-ray fan.
- `RaycastSensorLeft` / `RaycastSensorRight`: track-edge detection. Yawed ±45° and pitched 10° down.
- `EulerAnglesSensor`: orientation (yaw rate and roll).

**Actuators**
- `WheelColliderFrontLeft/Right`: steering and drive.
- `WheelColliderBackLeft/Right`: drive.
- `TransformWheel*`: visual wheel meshes, synced each step with `GetWorldPose`.

Detection layers are set up automatically when the scene starts: the children of the track parent are put on the `Road` layer and the children of the obstacle parent on the `Obs` layer. This keeps the raycasts for track following and obstacle avoidance separate.

---

## Control loop (`FixedUpdate`)

```
1. Update sensors     → yaw rate, wheel speed, speed control + slope, self-righting, anti-roll
2. Obstacle avoidance → highest priority (front ray fan)
3. Avoidance cooldown → stay in avoid mode briefly after the last hit
4. Track following    → only when not avoiding (side sensors)
5. Yaw damping        → only when not avoiding (smooths the steering)
6. Apply              → steering angle, motor torque, wheel visuals
```

---

## Tunable parameters

All of these can be adjusted in the Unity Inspector.

| Parameter | Default | Purpose |
|---|---|---|
| `maxMotorTorque` | 500 | Maximum drive torque |
| `maxSteerAngle` | 30° | Steering limit |
| `targetSpeed` | 7 km/h | Cruise speed |
| `frontRange` | 10 | Obstacle ray length |
| `trackRayLength` | 15 | Track sensor ray length |
| `SideSensorWidth` | 45° | Outward angle of the side sensors |
| `frontRayCount` / `frontFanAngle` | 5 / 20° | Obstacle fan resolution and spread |
| `steerLerpSpeed` / `avoidSteerLerpSpeed` | 1 / 1 | Steering smoothing |
| `yawDampingStrength` / `yawDampingWeight` | 0.015 / 0.6 | Yaw stabilisation strength |

---

## Getting started

1. Clone the repository:
   ```bash
   git clone https://github.com/anasir02/Self-Driving-Car-Simulation-Unity.git
   ```
2. Open the project folder in **Unity Hub**. Use Unity 6 or later, because the script uses `Rigidbody.linearVelocity`.
3. Make sure the layers **`Road`** and **`Obs`** exist under *Project Settings → Tags and Layers*.
4. Open the main scene and select the robot. In the Inspector, check that `TrackParent` and `ObstacleParent` are assigned.
5. Press **Play**. Turn on **Gizmos** in the Game view to see the sensor rays:
   - 🟢 green / 🔴 red: side sensor on track / off track
   - 🟡 yellow / 🔴 red: front fan clear / obstacle hit

---

## Project structure

```
Assets/
 └── Scripts/
      └── RobotController.cs   # All sensing, decision-making and control logic
Packages/
ProjectSettings/
```

---

## Tech stack

- **Unity** (WheelCollider physics, Rigidbody, Physics.Raycast)
- **C#**

---

## Author

**Nasir** · MSc Artificial Intelligence and Robotics, University of Hertfordshire
GitHub: [@anasir02](https://github.com/anasir02)
