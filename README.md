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

### 👉 [Wonikrobotics-git/allegro_hand_ros2_v4](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v4)

Use the [`humble` release](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v4/releases/tag/humble)
of the V4 controller with Ubuntu 22.04 and ROS 2 Humble. Set up and build the
controller before using this teleoperation code. Repository access is required
while the controller repository is private.

The repository name is `allegro_hand_ros2_v4`; ROS package names remain
unchanged, including `allegro_hand_bringup` and the `allegro_hand_ros2` metapackage.

### Setup Instructions

```bash
# Clone the Humble controller and build its packages
source /opt/ros/humble/setup.bash
mkdir -p ~/allegro_hand_v4_ws/src
cd ~/allegro_hand_v4_ws/src
git clone --branch humble https://github.com/Wonikrobotics-git/allegro_hand_ros2_v4.git
cd ~/allegro_hand_v4_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

See the [V4 user guide](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v4/blob/main/allegro_hand_bringup/getting_started_v4.md)
for detailed hardware setup. From this teleoperation repository, configure CAN
for your hand setup:

```bash
bash start_one_hand.bash         # For single hand
bash start_two_hands.bash        # For dual-hand setup
```

Then launch the controller from a terminal sourced with
`~/allegro_hand_v4_ws/install/setup.bash`:

```bash
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

> Reference: https://zhaohengyin.github.io/geort/

📖 **For details, see:** [GLOVE_BASED.md](./GLOVE_BASED.md)

---

### 2. Vision-Based Teleoperation

Camera-based hand pose estimation → retargeting to Allegro Hand.

> Reference: https://github.com/dexsuite/dex-retargeting

📖 **For details, see:** [VISION_BASED.md](./VISION_BASED.md)

---

## 📑 Referenced Research

- **GeoRT**: [https://zhaohengyin.github.io/geort/](https://zhaohengyin.github.io/geort/)

- **DexSuite – Dex-Retargeting**: [https://github.com/dexsuite/dex-retargeting](https://github.com/dexsuite/dex-retargeting)

---

## Future Expansion

- Expansion to additional Allegro Hand platforms
- Additional retargeting methods (using different hardware or algorithms)

---

## License

MIT License

- Modifications © 2025 **Wonik Robotics**
- Additional contributions © 2025 **Jungyeon Lee**

See the full license in the [LICENSE](./LICENSE) file.
