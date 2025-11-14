# Glove-Based Teleoperation

Real-time teleoperation of Allegro robotic hands using Manus gloves with rule-based or learning-based retargeting approaches.

This document covers two retargeting methods:
1. **Rule-Based Retargeting** - Heuristic mapping with immediate deployment
2. **GeoRT-Based Retargeting** - Learning-based approach for improved accuracy

---

## Glove Setup (Common for All Methods)

> **Note:** This project uses Manus Core 3 SDK v3.0.0. Later versions should work with minor adjustments.

### Prerequisites

- **MANUS Core 3 SDK** (with ROS2 Package): https://docs.manus-meta.com/3.0.0/Resources/
- **Connection Guide**: https://docs.manus-meta.com/3.0.0/Plugins/SDK/ROS2/getting%20started/

### Verification

After building, run `manus_data_viz.py` to verify glove connection:

```bash
python3 manus_data_viz.py
```

You can check the expected results in the [official visualization guide](https://docs.manus-meta.com/3.0.0/Plugins/SDK/ROS2/visualization/).

**Directory Structure**

```
ROS2/
├── manus_ros2
│   ├── client_scripts
│   │   └── manus_data_viz.py
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       ├── ClientLogging.hpp
│       ├── ClientPlatformSpecific.cpp
│       ├── ClientPlatformSpecific.hpp
│       ├── ClientPlatformSpecificTypes.hpp
│       ├── manus_data_publisher.cpp
│       ├── ManusDataPublisher.cpp
│       └── ManusDataPublisher.hpp
├── manus_ros2_msgs
└── ManusSDK
```

## Optional: Hardcode Left/Right Glove Recognition

By default, gloves are auto-assigned as `manus_glove_0`, `manus_glove_1` based on connection order. To ensure consistent left/right identification regardless of connection order, you can hardcode glove IDs.

### Implementation

Modify line 336 in `ManusDataPublisher.cpp`. For example, if your left glove ID is `123456789` and right glove ID is `-987654321`:

```cpp
auto t_Publisher = m_GlovePublisher.find(t_Msg.glove_id);
if(t_Publisher == m_GlovePublisher.end()){
    rclcpp::Publisher<manus_ros2_msgs::msg::ManusGlove>::SharedPtr t_NewPublisher;

    if(t_Msg.glove_id == 123456789){  // Replace with your left glove ID
        t_NewPublisher = this->create_publisher<manus_ros2_msgs::msg::ManusGlove>("manus_glove_left", 10);
    }
    else if (t_Msg.glove_id == -987654321){  // Replace with your right glove ID
        t_NewPublisher = this->create_publisher<manus_ros2_msgs::msg::ManusGlove>("manus_glove_right", 10);
    }
    else{  // Fallback for unknown gloves
        t_NewPublisher = this->create_publisher<manus_ros2_msgs::msg::ManusGlove>("manus_glove_" +
            std::to_string(m_GlovePublisher.size()), 10);
    }
    t_Publisher = m_GlovePublisher.emplace(t_Msg.glove_id, t_NewPublisher).first;
}
```

### Finding Your Glove IDs

1. Run the code and check logs (lines 348-357 print Glove ID every 10 seconds)
2. Wear each glove and note its ID to identify left/right
3. Replace the ID values in the code above with your actual IDs

### Benefits

- ✅ Consistent topic naming (`manus_glove_left`/`manus_glove_right`)
- ✅ Connection order independence
- ✅ Clearer data identification

---

## Method 1: Rule-Based Retargeting

Heuristic mapping approach for direct glove-to-robot joint transformation with Open3D visualization.

### Usage

#### Step 1: Publish Manus Glove Data

Start the Manus data publisher:

```bash
ros2 run manus_ros2 manus_data_publisher
```

#### Step 2: Run Teleoperation Script

> [!IMPORTANT]
> Before running, source the ROS2 workspace in the Manus SDK directory to access the glove topics:
> ```bash
> cd path/to/ManusSDK/ROS2
> source install/setup.bash
> ```

The main teleoperation script supports both single and dual robot setups.

**Basic Usage:**

```bash
# Single robot (default: controls right hand only)
python glove_based/rule_based_retargeting.py --setup single

# Dual robots (default: controls both hands)
python glove_based/rule_based_retargeting.py --setup dual
```

**Explicit Hand Selection:**

```bash
# Single robot, control left hand
python glove_based/rule_based_retargeting.py --setup single --hands left

# Dual robots, control only right hand
python glove_based/rule_based_retargeting.py --setup dual --hands right

# Dual robots, control only left hand
python glove_based/rule_based_retargeting.py --setup dual --hands left
```

**Command Summary:**

| Command | Result |
|---------|--------|
| `--setup single` | Control right hand only (default) |
| `--setup single --hands left` | Control left hand only |
| `--setup dual` | Control both hands (default) ✨ |
| `--setup dual --hands right` | Control right hand only |
| `--setup dual --hands left` | Control left hand only |

### Implementation Details

#### Main Transformation Function

The core retargeting logic in `rule_based_retargeting.py`:

```python

    def transform_glove_to_allegro(self, glove20, side):
        """
        Transform 20-dim glove ergonomics to 16-dim Allegro joint angles.

        Complete transformation pipeline:
        1. Extract finger values from glove data
        2. Map to Allegro joint angles in degrees
        3. Convert to radians
        4. Apply joint-specific scaling and offsets
        5. Clip to joint limits
        6. Apply EMA smoothing

        Args:
            glove20 (list): 20-dimensional glove ergonomics values
            side (str): Hand side ('left' or 'right')

        Returns:
            np.ndarray: 16-dimensional Allegro joint angles in radians (smoothed)
        """
        # ========================================================================
        # Step 1: Extract finger values from glove data
        # ========================================================================
        thumb_vals = np.array(glove20[0:4], dtype=float)
        index_vals = np.array(glove20[4:8], dtype=float)
        middle_vals = np.array(glove20[8:12], dtype=float)
        ring_vals = np.array(glove20[12:16], dtype=float)

        # ========================================================================
        # Step 2: Construct joint angles in degrees (order: thumb, index, middle, ring)
        # ========================================================================
        angle_deg = np.concatenate([
            # Thumb joints (4 DOF)
            [90 - 1.75 * thumb_vals[1]],     # Thumb CMC joint
            [-45 + 3.0 * thumb_vals[0]],     # Thumb base joint 1
            [-30 + 3.0 * thumb_vals[2]],     # Thumb base joint 2
            [thumb_vals[3]],                 # Thumb tip joint
            # Index finger (4 DOF)
            index_vals,
            # Middle finger (4 DOF: first joint +20°, then 3 more)
            [middle_vals[0] + 20],
            middle_vals[1:],
            # Ring finger (4 DOF: first 3, last +5°)
            ring_vals[0:3],
            [ring_vals[3] + 5],
        ])

        # ========================================================================
        # Step 3: Convert to radians
        # ========================================================================
        arr = np.deg2rad(angle_deg)

        # ========================================================================
        # Step 4: Apply joint-specific scaling and offsets
        # ========================================================================
        # Thumb scaling
        arr[0] *= 2.5                          # Joint 0: MCP Spread
        arr[1] = arr[1] * 2 + np.deg2rad(90)   # Joint 1: PIP Stretch + 90° offset
        arr[3] *= 2                            # Joint 3: DIP Stretch

        # Index finger scaling
        arr[4] *= -0.5   # Joint 4: MCP Spread
        arr[5] *= 1.5    # Joint 5: MCP Stretch
        arr[7] *= 2      # Joint 7: PIP Stretch

        # Middle finger scaling
        arr[8] *= -0.2   # Joint 8: MCP Spread
        arr[9] *= 1.5    # Joint 9: MCP Stretch
        arr[11] *= 2     # Joint 11: PIP Stretch

        # Ring finger scaling
        arr[12] *= 0.1   # Joint 12: MCP Spread
        arr[13] *= 1.5   # Joint 13: MCP Stretch
        arr[15] *= 2     # Joint 15: PIP Stretch

        # ========================================================================
        # Step 5: Clip to joint limits
        # ========================================================================
        arr = np.clip(arr, ALLEGRO_LOWER_LIMITS, ALLEGRO_UPPER_LIMITS)

        # ========================================================================
        # Step 6: Apply exponential moving average (EMA) smoothing
        # ========================================================================
        side_key = side.lower()
        prev_arr = self.prev_arr.get(side_key)

        if prev_arr is None:
            smoothed = arr.copy()
        else:
            smoothed = self.alpha * arr + (1.0 - self.alpha) * prev_arr

        self.prev_arr[side_key] = smoothed

        return smoothed

```

#### Transformation Pipeline Details

The retargeting process consists of 6 steps:

1. **Extract finger values** - Parse 20-dim glove data into finger segments (thumb, index, middle, ring)
2. **Map to joint angles** - Apply heuristic mapping rules to convert glove measurements to Allegro joint angles (degrees)
3. **Convert to radians** - Transform all angles to radians for ROS2 compatibility
4. **Apply scaling** - Apply finger-specific scaling factors and offsets for optimal mapping
5. **Clip to limits** - Ensure all joint values are within safe hardware limits
6. **Smooth output** - Apply exponential moving average (EMA) filter to reduce jitter

---

## Method 2: GeoRT-Based Retargeting

Learning-based approach using GeoRT (Geometric Retargeting) for improved accuracy through data-driven mapping.

### Workflow

1. **Log Hand Data** - Record human hand poses from glove sensors
2. **Gather Robot Data** - Collect corresponding robot configurations
3. **Train Model** - Learn geometric mapping between human and robot hands
4. **Inference in Simulation** - Test learned model in simulated environment
5. **Deploy to Real Robot** - Transfer trained model to physical hardware

> **Note:** GeoRT implementation details coming soon.
