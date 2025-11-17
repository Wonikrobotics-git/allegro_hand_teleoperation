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
│       ├── manus_data_publisher.cpp
│       ├── ...
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

1. **Setup Environment** - Create conda environment with required dependencies
2. **Log Hand Data** - Record human hand poses from glove sensors
3. **Generate Robot Data** - Collect robot kinematic configurations
4. **Train Model** - Learn geometric mapping between human and robot hands
5. **Inference & Deployment** - Test and deploy to simulation/hardware

---

### Step 0: Setup Conda Environment

**System Requirements**

- Ubuntu 22.04
- ROS2 Humble
- Allegro Hand V4
- Sapien Simulator 2.2.2
- Python 3.10+

**Installation**

```bash
# Create conda environment
conda create -n geort python=3.10

# Activate environment
conda activate geort

# Install dependencies
pip install -r glove_based/geort_requirements.txt

# Install GeoRT package
cd glove_based
pip install -e .
```

---

### Step 1: Log Human Hand Data

Record human hand motions using the Manus glove for training the retargeting model.

**1.1 Start Manus Data Publisher**

```bash
ros2 run manus_ros2 manus_data_publisher
```

**1.2 Preprocess Glove Data**

Convert raw glove data to 21-joint skeleton format compatible with GeoRT:

```bash
conda activate geort
python glove_based/manus_skeleton_21.py
```

**1.3 Log Human Data**

Record hand motions with specified parameters:

```bash
# Right hand example
python glove_based/geort_data_logger.py --name human1 --handness right --duration 60 --hz 30

# Left hand example
python glove_based/geort_data_logger.py --name human1 --handness left --duration 60 --hz 30
```

**Parameters:**
- `--name`: Human identifier (default: `human1`)
- `--handness`: Hand side (`left` or `right`, default: `right`)
- `--duration`: Recording duration in seconds (default: 60)
- `--hz`: Target frame rate (default: 30)

**Output:** Data saved to `glove_based/data/{name}_{handness}_{timestamp}.npy`

---

### Step 2: Generate Robot Data

Generate robot kinematics dataset for the Allegro hand configuration.


```bash
conda activate geort

# Generate and save dataset (default)
python glove_based/geort/generate_robot_data.py --hand allegro_left

# Visualize only (quick test, no dataset)
python glove_based/geort/generate_robot_data.py --hand allegro_left --viz

# Preview mode (generate small dataset + visualize, no save)
python glove_based/geort/generate_robot_data.py --hand allegro_left --num-samples 100 --no-save
```

**Common Usage Patterns**

| Command | Description | Generates | Saves | Visualizes |
|---------|-------------|-----------|-------|------------|
| `--hand allegro_left` | Standard generation | 1M samples | ✅ | ❌ |
| `--hand allegro_left --viz` | Visualization only | - | ❌ | ✅ 100 configs |
| `--hand allegro_left --num-samples 100 --no-save` | Preview mode | 100 samples | ❌ | ✅ 100 configs |
| `--hand allegro_left --num-samples 50000 --viz --save` | Generate + visualize | 50K samples | ✅ | ✅ 100 configs |

**Compact Command Options**

```bash
# Full example with all options
python glove_based/geort/generate_robot_data.py \
    --hand allegro_right \        # hand configuration
    --num-samples 100000 \        # number of samples
    --viz \                       # enable visualization
    --save \                      # save dataset (with --viz)
    --interval 0.15               # visualization interval (seconds)
```

**Parameters:**

| Option | Description | Default |
|--------|-------------|---------|
| `--hand` | Hand config (`allegro_left` or `allegro_right`) | `allegro_right` |
| `--num-samples` | Number of samples | 1M (generate) / 100 (viz) |
| `--viz` | Enable visualization | `False` |
| `--save` | Save dataset (use with `--viz`) | Auto |
| `--no-save` | Don't save (implies `--viz`) | `False` |
| `--interval` | Viz interval in seconds | `0.1` |


**Output:** `glove_based/data/allegro_{left|right}.npz`

---

### Step 3: Train GeoRT Model

Train the geometric retargeting model using logged human data.

**Basic Training**

```bash
# Right hand
python glove_based/geort/trainer.py \
    --hand allegro_right \
    --human_data human1_right_1028_150817.npy

# Left hand
python glove_based/geort/trainer.py \
    --hand allegro_left \
    --human_data human1_left_1028_150409.npy
```

**Advanced Training**

```bash
python glove_based/geort/trainer.py \
    --hand allegro_right \
    --human_data human1_right_1117_105023.npy \
    --ckpt_tag "experiment_v1" \
    --w_chamfer 80.0 \
    --w_curvature 0.1 \
    --w_pinch 1.0 \
    --wandb_project my_geort_project \
    --wandb_entity my_username
```

**Training Parameters:**

*Required:*
- `--hand`: Robot hand config (`allegro_left` or `allegro_right`)
- `--human_data`: Human data filename (in `glove_based/data/`)

*Optional Loss Weights:*
- `--w_chamfer`: Chamfer loss weight (default: 80.0)
- `--w_curvature`: Curvature loss weight (default: 0.1)
- `--w_collision`: Collision loss weight (default: 0.0)
- `--w_pinch`: Pinch loss weight (default: 1.0)

*Wandb Configuration:*
- `--wandb_project`: Project name (default: `geort`)
- `--wandb_entity`: Username/team (optional)
- `--no_wandb`: Disable wandb logging
- `--ckpt_tag`: Checkpoint tag (default: `''`)

**Output:**
- Checkpoints: `glove_based/checkpoint/allegro_{left|right}_{timestamp}_{tag}/`
- Monitor: Wandb dashboard or terminal output

---

### Step 4: Inference & Deployment

#### 4.1 Replay Evaluation (Recorded Data)

Test trained model with pre-recorded human hand data in Sapien simulator.

**Right Hand**

```bash
python glove_based/geort_replay_evaluation.py \
    --ckpt "human1_right_1028_150817_allegro_right_last" \
    --hand allegro_right \
    --data human1_right_1028_150817.npy
```

**Left Hand**

```bash
python glove_based/geort_replay_evaluation.py \
    --ckpt "human1_left_1028_150409_allegro_left_last" \
    --hand allegro_left \
    --data human1_left_1028_150409.npy
```

---

#### 4.2 Real-time Simulation (Sapien)

Test trained model with live glove input in Sapien simulator.

**Setup**

```bash
# Terminal 1: Start Manus data publisher
ros2 run manus_ros2 manus_data_publisher

# Terminal 2: Preprocess glove data
python glove_based/manus_skeleton_21.py
```

**Run Real-time Evaluation**

```bash
# Terminal 3: Right hand
conda activate geort
python glove_based/geort_realtime_evaluation.py \
    --ckpt "human1_right_1028_150817_allegro_right_last" \
    --hand allegro_right

# OR Left hand
python glove_based/geort_realtime_evaluation.py \
    --ckpt "human1_left_1028_150409_allegro_left_last" \
    --hand allegro_left
```

---

#### 4.3 Real Hardware Deployment

Deploy trained model to physical Allegro hands using the `GeortAllegroDeployer` ROS2 node.

**Architecture Overview**

The deployment pipeline consists of:
1. **Manus Mocap Node**: Streams hand pose data from Manus gloves
2. **Manus Skeleton Preprocessor**: Converts glove data to 21-joint skeleton format
3. **GeoRT Deployer**: Runs trained models and publishes commands to robot controllers
   - Loads trained IK models for left and right hands
   - Subscribes to preprocessed hand poses
   - Performs forward pass through GeoRT model
   - Applies post-processing (reordering, optional calibration)
   - Publishes joint commands to Allegro hand controllers

**Setup**

```bash
# Terminal 1: Start Manus data publisher
ros2 run manus_ros2 manus_data_publisher

# Terminal 2: Preprocess glove data
conda activate geort
python glove_based/manus_skeleton_21.py

# Terminal 3: Launch Allegro hand controller
cd allegro_hand_ros2
source install/setup.bash
ros2 launch allegro_hand_bringup allegro_hand_duo.launch.py
```

**Run GeoRT Deployment**

```bash
# Terminal 4: Load Both Hand Checkpoints
python glove_based/geort_allegro_deploy.py \
      --right_ckpt "human1_right_1028_150817_allegro_right_last" \
      --left_ckpt "human1_left_1028_150409_allegro_left_last"
```

**Post-Processing Notes**

The `GeortAllegroDeployer` includes a `post_processing_commands()` function that:
- **Step 1**: Reorders joints from model output (Index, Middle, Ring, Thumb) to hardware order (Thumb, Index, Middle, Ring)
- **Step 2**: Applies optional per-joint calibration adjustments (currently disabled by default)

> **Important**: The Step 2 calibration adjustments are hardware-specific tweaks that are **optional and not recommended** for general use. By default, these are commented out in the code. Only enable if you observe systematic errors in your specific robot setup.

**Command Arguments**

| Argument | Description | Required |
|----------|-------------|----------|
| `--right_ckpt` | Checkpoint tag for right hand model | Yes |
| `--left_ckpt` | Checkpoint tag for left hand model | Yes |
| `--loop_hz` | Control loop frequency in Hz | No (default: 100.0) |
