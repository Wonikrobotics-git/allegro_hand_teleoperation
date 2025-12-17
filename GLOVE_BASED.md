# Glove-Based Teleoperation

Real-time teleoperation of Allegro robotic hands using Manus gloves with rule-based or learning-based retargeting approaches.

This document covers two retargeting methods:
1. **Rule-Based Retargeting** - Heuristic mapping with immediate deployment
2. **GeoRT-Based Retargeting** - Learning-based approach for improved accuracy

---

## Glove Setup (Common for All Methods)

> **Note:** This project uses Manus Core 3 SDK v3.0.0. Later versions should work with minor adjustments.

> [!IMPORTANT]
> **Calibration Recommendation:** For stability, we recommend performing glove calibration on Windows using the MANUS Core application (v3.0.1 or later) and saving the `.mcal` files. These calibration files can then be transferred to your Linux system for use with the ROS2 node. See [Glove Calibration](#glove-calibration) section below for details.

### Prerequisites

- **MANUS Core 3 SDK** (with ROS2 Package): https://docs.manus-meta.com/3.0.0/Resources/
- **Connection Guide**: https://docs.manus-meta.com/3.0.0/Plugins/SDK/ROS2/getting%20started/

### Glove Calibration

Calibration files (`.mcal`) store **skeleton scaling and sensor correction parameters** matched to the user's hand size:

| Item | Without Calibration | With Calibration |
|------|---------------------|------------------|
| Finger length | SDK default values | Scaled to actual user hand size |
| Joint angles | Based on raw sensor data | Corrected for individual ROM |
| Fingertip position | May be inaccurate | Millimeter precision |
| Hand pose reproducibility | Low | High |

#### How Calibration is Applied in ROS2

When the `manus_data_publisher` node runs, calibration files are loaded into the MANUS SDK, and **all published data is output in a calibrated state**.

```
MANUS Glove (Hardware)
        ↓
   Raw Sensor Data
        ↓
┌─────────────────────────────────────────────┐
│  MANUS SDK (CoreSdk_SetGloveCalibration)    │
│  ← humanLeftMetaglove.mcal loaded           │
│  ← humanRightMetaglove.mcal loaded          │
│  (calibration files exported from Windows)  │
│                                             │
│  Skeleton calculation with calibration      │
└─────────────────────────────────────────────┘
        ↓
   Calibrated Data
        ↓
┌─────────────────────────────────────────────┐
│  manus_data_publisher (ROS2 Node)           │
│  → /manus_glove_left                        │
│  → /manus_glove_right                       │
└─────────────────────────────────────────────┘
        ↓
   ROS2 Topics (calibration-applied data)
```

#### Calibration Application in Code

**1. Calibration file loading and SDK application** (`ManusDataPublisher.cpp:593-598`):
```cpp
SDKReturnCode t_Res = CoreSdk_SetGloveCalibration(
    t_GloveId,
    t_CalibData->data(),      // Binary data from .mcal file
    static_cast<uint32_t>(t_CalibData->size()),
    &t_Result
);
```

**2. Receiving calibrated skeleton data** (`ManusDataPublisher.cpp:655`):
```cpp
// SDK internally returns data with calibration applied
CoreSdk_GetRawSkeletonData(i, t_NxtClientRawSkeleton.nodes.data(), ...);
```

**3. Publishing as ROS2 messages** (`ManusDataPublisher.cpp:337-351`):
```cpp
// These position/orientation values have calibration applied
t_Pose.position.x = t_Pos.x;
t_Pose.position.y = t_Pos.y;
t_Pose.position.z = t_Pos.z;
t_Pose.orientation.x = t_Rot.x;
// ...
t_Msg.raw_nodes.push_back(t_Node);
```

#### ROS2 Message Fields with Calibration Applied

The following fields in the `ManusGlove` message contain calibration-corrected values:
- `raw_nodes[].pose.position` - Position of each finger joint (scaled to hand size via calibration)
- `raw_nodes[].pose.orientation` - Rotation of each finger joint (joint angle corrected via calibration)
- `ergonomics[].value` - Finger flexion/extension values (corrected to individual ROM via calibration)

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

## Hardcode Left/Right Glove Recognition

> [!CAUTION]
> **This step is practically required for this project.** By default, gloves are published as `manus_glove_0`, `manus_glove_1` based on connection order. However, all Python scripts in this project subscribe to `manus_glove_left` and `manus_glove_right` topics. **If you skip this hardcoding step, the scripts will run without errors but will not receive any glove data**, resulting in no robot movement or response.

### Step 1: Find Your Glove IDs

After building the Manus ROS2 packages, run the data publisher:

```bash
ros2 run manus_ros2 manus_data_publisher
```

The Glove ID is automatically printed to the logs every 10 seconds. Wear each glove one at a time and note its ID from the logs to identify left/right.

### Step 2: Modify the Code

Modify line 336 in `ManusDataPublisher.cpp`. Replace the example IDs with your actual glove IDs:

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

After modifying, rebuild the Manus ROS2 packages to apply the changes.

### Result

Once configured, gloves will be published to consistent topic names (`manus_glove_left`/`manus_glove_right`) regardless of connection order, enabling all scripts in this project to work correctly.

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

<p align="center">
  <img src="./materials/data_logger.gif" width="60%"/>
</p>

---

### Step 2: Generate Robot Data

Generate robot kinematics dataset for the Allegro hand configuration.

```bash
conda activate geort

# Standard: Generate 1M samples and save (recommended)
python glove_based/geort/generate_robot_data.py --hand allegro_left

# Optional: Visualize only (no dataset saved)
python glove_based/geort/generate_robot_data.py --hand allegro_left --viz
```

**Key Parameters:**
- `--hand`: Hand config (`allegro_left` or `allegro_right`)
- `--num-samples`: Number of samples (default: 1M for generation, 100 for viz)
- `--viz`: Enable visualization mode
- `--no-save`: Preview mode (visualize without saving)

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
- **Checkpoints**: `glove_based/checkpoint/{human_name}_{robot_name}_{timestamp}_{tag}/`
  - `best.pth` - Model with lowest training loss (automatically tracked, **recommended for deployment**)
  - `last.pth` - Latest model from final epoch
  - `epoch_{N}.pth` - Periodic snapshots (every 100 epochs)
  - `config.json` - Training configuration
- **Monitor**: Wandb dashboard or terminal output
- **Chamfer Visualization**: `glove_based/data/chamfer_{human_name}_{robot_name}.html`
  - Interactive 3D point cloud visualization for qualitative assessment
  - Open in browser to inspect human-robot hand geometry alignment
  - Use with quantitative metrics (loss values) for comprehensive evaluation
![](./materials/chamfer_distance.png)


**Training Notes:**

*Best Checkpoint Tracking:* Training automatically saves the model with lowest loss as `best.pth` (recommended). Console shows: `→ New best model saved! Loss: X.XXXXe-XX`. All evaluation/deployment scripts use `best.pth` by default; add `--use_last` flag to use `last.pth` instead.

---

### Step 4: Inference & Deployment

#### 4.1 Replay Evaluation (Recorded Data)

Test trained model with pre-recorded human hand data in Sapien simulator.

**Right Hand**

```bash
python glove_based/geort_replay_evaluation.py \
    --ckpt "human1_right_1028_150817_allegro_right_s10" \
    --hand allegro_right \
    --data human1_right_1028_150817.npy
```

**Left Hand**

```bash
python glove_based/geort_replay_evaluation.py \
    --ckpt "human1_left_1028_150409_allegro_left_s10" \
    --hand allegro_left \
    --data human1_left_1028_150409.npy
```

**Parameters:**
- `--ckpt`: Checkpoint tag for trained model
- `--hand`: Hand configuration (`allegro_left` or `allegro_right`)
- `--data`: Human data filename (in `glove_based/data/`)
- `--use_last`: Load last checkpoint instead of best (optional, default: best)

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
    --ckpt "human1_right_1028_150817_allegro_right_s10" \
    --hand allegro_right

# OR Left hand
python glove_based/geort_realtime_evaluation.py \
    --ckpt "human1_left_1028_150409_allegro_left_s10" \
    --hand allegro_left
```

**Parameters:**
- `--ckpt`: Checkpoint tag for trained model
- `--hand`: Hand configuration (`allegro_left` or `allegro_right`)
- `--use_last`: Load last checkpoint instead of best (optional, default: best)

---

#### 4.3 Real Hardware Deployment

Deploy trained model to physical Allegro hands using GeoRT deployer ROS2 nodes.

**Architecture Overview**

The deployment pipeline consists of:
1. **Manus Mocap Node**: Streams hand pose data from Manus gloves
2. **Manus Skeleton Preprocessor**: Converts glove data to 21-joint skeleton format
3. **GeoRT Deployer**: Runs trained models and publishes commands to robot controllers
   - Loads trained IK models for left and/or right hands
   - Subscribes to preprocessed hand poses (`/manus_poses` or `/manus_poses_{left|right}`)
   - Uses `AllegroCommandForwarder` for controller management:
     - Automatically activates appropriate controllers via `controller_manager`
     - Single robot mode (`two_robots=False`): `/allegro_hand_position_controller/commands`
     - Dual robot mode (`two_robots=True`): `/allegro_hand_position_controller_{r|l}/commands`
   - Performs forward pass through GeoRT model
   - Applies post-processing (reordering, optional calibration)
   - Applies EMA smoothing based on previous command for stable control
   - Applies rate limiting to prevent sudden joint movements
   - Publishes joint commands to Allegro hand controllers
   - Returns to safe base position on shutdown

**Smoothing Mechanism:**
- **EMA Formula**: `smoothed = α × target + (1 - α) × previous_command`
- **α (smoothing_alpha)**: Controls responsiveness (0 = maximum smoothing, 1 = no smoothing)
- **Rate Limiting**: Limits maximum joint angle change per control tick

**Configure PD Gains:**

Before launching the controller, configure the PD gains for optimal performance with this deployment. Edit the PD gains configuration file in your `allegro_hand_ros2` workspace:

```yaml
# File: allegro_hand_ros2/allegro_hand_hardwares/v4/description/config/pd_gains.yaml
p_gains:
  joint00: 1.5
  ...
  joint33: 1.5
d_gain: 
  joint00: 0.1024
  ...
  joint33: 0.1024
```

Reference configuration: [pd_gains.yaml](https://github.com/Wonikrobotics-git/allegro_hand_ros2/blob/main/allegro_hand_hardwares/v4/description/config/pd_gains.yaml)

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

# For single hand
ros2 launch allegro_hand_bringup allegro_hand.launch.py

# For dual hands
ros2 launch allegro_hand_bringup allegro_hand_duo.launch.py
```

---

**Option A: Single Hand Deployment**

Deploy to a single Allegro hand using `geort_allegro_deploy_single.py`.

> **Note:** This script uses the same `GeortAllegroDeployer` class from `geort_allegro_deploy.py`, ensuring consistent behavior across single and dual hand deployments.

**Basic Usage:**

```bash
# Right hand deployment
python glove_based/geort_allegro_deploy_single.py \
    --ckpt "human1_right_1028_150817_allegro_right_s10" \
    --side right

# Left hand deployment
python glove_based/geort_allegro_deploy_single.py \
    --ckpt "human1_left_1028_150409_allegro_left_s10" \
    --side left
```

**Advanced Usage:**

```bash
# Deploy with custom smoothing and mocap topic
python glove_based/geort_allegro_deploy_single.py \
    --ckpt "human1_right_1028_150817_allegro_right_s10" \
    --side right \
    --smoothing_alpha 0.7 \
    --loop_hz 100.0 \
    --mocap_topic "/manus_poses_right"
```

**Command Arguments:**

| Argument | Description | Required | Default |
|----------|-------------|----------|---------|
| `--ckpt` | Checkpoint tag for the hand model | Yes | - |
| `--side` | Hand side (left or right) | Yes | - |
| `--mocap_topic` | Manus mocap topic name | No | `/manus_poses_{side}` |
| `--loop_hz` | Control loop frequency in Hz | No | 100.0 |
| `--smoothing_alpha` | EMA smoothing alpha (0..1, 1=no smoothing) | No | 0.9 |
| `--use_last` | Load last checkpoint instead of best | No | False (uses best) |

> **Note:** Single hand deployment uses `AllegroCommandForwarder` with `two_robots=False`, which automatically publishes to `/allegro_hand_position_controller/commands` (no suffix) and activates the controller, suitable for single robot setups.

**Additional ROS2 Parameters:**

The deployer also supports runtime-configurable ROS2 parameters:
- `smoothing_alpha`: EMA smoothing factor (0..1)
- `max_delta`: Maximum joint angle change per tick in radians (0 disables)
- `snap_on_start`: Initialize to target pose directly on first frame
- `loop_hz`: Control loop frequency

---

**Option B: Dual Hand Deployment**

Deploy to both Allegro hands simultaneously using `geort_allegro_deploy.py`.

**Basic Usage:**

```bash
# Deploy both hands
python glove_based/geort_allegro_deploy.py \
    --right_ckpt "human1_right_1028_150817_allegro_right_s10" \
    --left_ckpt "human1_left_1028_150409_allegro_left_s10"
```


**Advanced Usage:**

```bash
# Deploy with custom smoothing
python glove_based/geort_allegro_deploy.py \
    --right_ckpt "human1_right_1028_150817_allegro_right_s10" \
    --left_ckpt "human1_left_1028_150409_allegro_left_s10" \
    --smoothing_alpha 0.7 \
    --loop_hz 100.0
```

**Command Arguments:**

| Argument | Description | Required | Default |
|----------|-------------|----------|---------|
| `--right_ckpt` | Checkpoint tag for right hand model | Yes | - |
| `--left_ckpt` | Checkpoint tag for left hand model | Yes | - |
| `--loop_hz` | Control loop frequency in Hz | No | 100.0 |
| `--smoothing_alpha` | EMA smoothing alpha (0..1, 1=no smoothing) | No | 0.9 |
| `--use_last` | Load last checkpoints instead of best | No | False (uses best) |

**Additional ROS2 Parameters:**

The deployer also supports runtime-configurable ROS2 parameters:
- `smoothing_alpha`: EMA smoothing factor (0..1)
- `max_delta`: Maximum joint angle change per tick in radians (0 disables)
- `snap_on_start`: Initialize to target pose directly on first frame
- `loop_hz`: Control loop frequency

---

**Post-Processing Notes**

Both deployment options use the same `GeortAllegroDeployer` class (defined in `geort_allegro_deploy.py`), ensuring consistent behavior. The class includes a `post_processing_commands()` function that:
- **Step A**: Reorders joints from model output (Index, Middle, Ring, Thumb) to hardware order (Thumb, Index, Middle, Ring)
- **Step B**: Applies optional per-joint calibration adjustments (currently disabled by default)

> **Important**: The Step B calibration adjustments are hardware-specific tweaks that are **optional and not recommended** for general use. By default, these are commented out in the code. Only enable if you observe systematic errors in your specific robot setup.

**Implementation Details:**

Both deployment scripts use the same `GeortAllegroDeployer` class with `AllegroCommandForwarder` for controller management:

- **Single deployment** (`geort_allegro_deploy_single.py`):
  - Imports `GeortAllegroDeployer` class
  - Uses `AllegroCommandForwarder(side=side, two_robots=False)`
  - Controller: `/allegro_hand_position_controller/commands` (no suffix)
  - Automatically activates the controller via controller_manager
  - Suitable for single robot setups

- **Dual deployment** (`geort_allegro_deploy.py`):
  - Defines `GeortAllegroDeployer` class
  - Uses `AllegroCommandForwarder(side=side, two_robots=True)`
  - Controllers: `/allegro_hand_position_controller_r/commands` and `/allegro_hand_position_controller_l/commands`
  - Automatically activates both controllers via controller_manager
  - Suitable for dual robot setups with separate controllers

- **Shared components**:
  - Identical smoothing, rate limiting, and post-processing logic
  - Controller activation and management via `AllegroCommandForwarder`
  - Safe base position return on shutdown
