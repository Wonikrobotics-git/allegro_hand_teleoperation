# allegro_hand_teleoperation

Teleoperation Application Code for the Allegro Hand Platforms

This repository provides a teleoperation framework for the Allegro Hand (V4 series).
The core feature is **hand retargeting**, which converts glove-based or vision-based inputs into Allegro Hand joint commands.

> [!NOTE]
> This project provides **Hand Teleoperation code only**.
> **Robot Arm Teleoperation must be developed separately.**

---

## Dependency: Allegro Hand ROS2 Controller

This project operates based on the official controller:

### 👉 [Wonikrobotics-git/allegro_hand_ros2](https://github.com/Wonikrobotics-git/allegro_hand_ros2)

Make sure to set up the Allegro Hand ROS2 controller before using this teleoperation code.

### Setup Instructions

```bash
# Build the package
colcon build --packages-select allegro_hand_ros2

# Source the workspace
source install/setup.bash

# CAN setup
./start_one_hand.sh              # For single hand
./start_two_hands.sh             # For dual-hand setup

# Run the controller
ros2 launch allegro_hand_bringup allegro_hand.launch.py      # Single hand
ros2 launch allegro_hand_bringup allegro_hand_duo.launch.py  # Dual hands
```

### Controller Topic Names

> [!IMPORTANT]
> Controller command topics differ based on setup:
> - **Single hand:** `allegro_hand_position_controller/commands`
> - **Dual hands:** `allegro_hand_position_controller_r/commands` and `allegro_hand_position_controller_l/commands`
>
> Ensure your teleoperation code uses the correct topic names.

---

## Contents Overview

### 1. Glove-Based Teleoperation (Manus Glove)

#### 1-1. Rule-Based Retargeting
Simple heuristic mapping approach for joint transformation.

#### 1-2. GeoRT-Inspired Retargeting
AI-based mapping inspired by the GeoRT architecture.
Reference: https://zhaohengyin.github.io/geort/

📖 **For details, see:** [GLOVE_BASED.md](./GLOVE_BASED.md)

---

### 2. Vision-Based Teleoperation

Camera-based hand pose estimation → retargeting to Allegro Hand.
Reference: https://github.com/dexsuite/dex-retargeting

📖 **For details, see:** [VISION_BASED.md](./VISION_BASED.md)

---

## Referenced Research

- **GeoRT**
  https://zhaohengyin.github.io/geort/
  (CC-BY-NC / Code not included)

- **DexSuite – Dex-Retargeting**
  https://github.com/dexsuite/dex-retargeting
  (Reference for concepts)

---

## Future Expansion

- Expansion to additional Allegro Hand platforms
- Additional retargeting methods (using different hardware or algorithms)

---

## License

MIT License

- Modifications © 2025 **WonikRobotics_official**
- Additional contributions © 2025 **Jungyeon Lee**

See the full license in the [LICENSE](./LICENSE) file.
