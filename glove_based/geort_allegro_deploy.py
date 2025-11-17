#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

import time
import math
import argparse
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from manus_mocap import ManusMocap
from allegro import AllegroCommandForwarder
from geort import load_model

class GeortAllegroDeployer(Node):
    def __init__(self, side: str, mocap: ManusMocap, model, loop_hz: float = 50.0):
        super().__init__(f'manus_allegro_deployer_{side}')
        self.mocap = mocap
        self.model = model
        self.side = side
        self.allegro = AllegroCommandForwarder(side=side)

        # Parameters (declarable/tweakable)
        self.declare_parameter('smoothing_alpha', 0.9)   # EMA alpha: 0..1 (1 = no smoothing)
        self.declare_parameter('max_delta', 0.05)        # rad per tick max change (0 disables)
        self.declare_parameter('snap_on_start', True)    # Whether to initialize to target directly on the first frame
        self.declare_parameter('loop_hz', float(loop_hz))

        self.alpha = float(self.get_parameter('smoothing_alpha').value)
        self.max_delta = float(self.get_parameter('max_delta').value)
        self.snap_on_start = bool(self.get_parameter('snap_on_start').value)

        # internal state
        self._last_cmd = None   # numpy array shape (16,)
        self._initialized = False

        # Timer
        timer_period = 1.0 / float(self.get_parameter('loop_hz').value)
        self.create_timer(timer_period, self._on_timer)

    def post_processing_commands(self, qpos):
        """
        Post-process model output: reorder joints and apply calibration tweaks.

        Args:
            qpos: Model output with 16 joint angles in model order
                  [Index(0:4), Middle(4:8), Ring(8:12), Thumb(12:16)]

        Returns:
            numpy array (16,) in Allegro hardware order
            [Thumb(0:4), Index(4:8), Middle(8:12), Ring(12:16)]
        """
        qpos = np.asarray(qpos).flatten()

        # ========================================================================
        # Step 1: Reorder from model output to hardware order
        # ========================================================================
        # Model order: Index, Middle, Ring, Thumb
        # Hardware order: Thumb, Index, Middle, Ring
        try:
            allegro_hw = np.concatenate([
                qpos[12:16],  # Thumb (joints 0-3)
                qpos[0:4],    # Index (joints 4-7)
                qpos[4:8],    # Middle (joints 8-11)
                qpos[8:12]    # Ring (joints 12-15)
            ])
        except Exception as e:
            self.get_logger().error(f"{self.side} reordering error: {e}")
            return np.zeros(16, dtype=float)

        allegro_hw = allegro_hw.astype(float)

        # ========================================================================
        # Step 2: Apply per-joint calibration adjustments (OPTIONAL - NOT RECOMMENDED)
        # ========================================================================
        # WARNING: These adjustments are hardware-specific calibration tweaks.
        # In most cases, you should skip this step and use the model output directly.
        # These values were tuned for a specific robot setup and may not generalize well.
        # Consider removing or modifying these only if you observe systematic errors
        # in your specific deployment.
        #
        # Joint numbering (hardware order):
        # Thumb:  [0]=MCP_Spread, [1]=MCP_Stretch, [2]=PIP, [3]=DIP
        # Index:  [4]=MCP_Spread, [5]=MCP_Stretch, [6]=PIP, [7]=DIP
        # Middle: [8]=MCP_Spread, [9]=MCP_Stretch, [10]=PIP, [11]=DIP
        # Ring:   [12]=MCP_Spread, [13]=MCP_Stretch, [14]=PIP, [15]=DIP

        # # --- Thumb adjustments ---
        # allegro_hw[0] = allegro_hw[0] * 1.3 + (20.0 * math.pi / 180.0)  # MCP_Spread: scale + offset
        # allegro_hw[1] = allegro_hw[1] * 1.4                              # MCP_Stretch: scale only
        # allegro_hw[3] = allegro_hw[3] - (10.0 * math.pi / 180.0)        # DIP: offset

        # # --- Index adjustments ---
        # allegro_hw[7] = 0.0  # DIP: fixed position

        # # --- Middle adjustments ---
        # allegro_hw[11] = 0.0  # DIP: fixed position

        # # --- Ring adjustments ---
        # allegro_hw[15] = 0.0  # DIP: fixed position

        return allegro_hw

    def _on_timer(self):
        data = self.mocap.get()
        if data is None:
            # no mocap frame available
            return

        points = data.get('result', None)
        if points is None:
            return

        # forward through model (protect against model errors)
        try:
            qpos = self.model.forward(points)
        except Exception as e:
            self.get_logger().error(f"{self.side} model forward failed: {e}")
            return

        target = self.post_processing_commands(qpos)  # numpy (16,)

        # initialize last_cmd on first valid frame
        if not self._initialized:
            if self.snap_on_start:
                self._last_cmd = target.copy()
            else:
                self._last_cmd = np.zeros_like(target)
            self._initialized = True
            # publish initial value immediately (avoid initial silence)
            msg = Float64MultiArray()
            msg.data = self._last_cmd.tolist()
            self.allegro.publisher_.publish(msg)
            self.get_logger().info(f"{self.side} initialized cmd: {np.round(self._last_cmd,3).tolist()}")
            return

        # EMA smoothing
        alpha = self.alpha
        smoothed = alpha * target + (1.0 - alpha) * self._last_cmd

        # rate limit per joint
        if self.max_delta and self.max_delta > 0.0:
            delta = smoothed - self._last_cmd
            # clamp
            delta = np.clip(delta, -self.max_delta, self.max_delta)
            new_cmd = self._last_cmd + delta
        else:
            new_cmd = smoothed

        # publish
        msg = Float64MultiArray()
        msg.data = new_cmd.tolist()
        self.allegro.publisher_.publish(msg)

        # update internal state
        self._last_cmd = new_cmd

        # small debug for thumb only (in degrees for readability)
        thumb = new_cmd[:4]
        thumb_deg = np.array(thumb) * 180.0 / math.pi
        self.get_logger().debug(f"{self.side} Thumb(deg): {thumb_deg.round(1).tolist()}")

def main():
    parser = argparse.ArgumentParser(
        description='Deploy GeoRT models to Allegro hands with Manus gloves',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Example:
  # Deploy both hands with trained checkpoints
  %(prog)s --right_ckpt "human1_right_1028_150817_allegro_right_last" \\
           --left_ckpt "human1_left_1028_150817_allegro_left_last"
        ''')

    parser.add_argument('--right_ckpt', type=str, required=True,
                        help="Checkpoint tag for right hand model")
    parser.add_argument('--left_ckpt', type=str, required=True,
                        help="Checkpoint tag for left hand model")
    parser.add_argument('--loop_hz', type=float, default=100.0,
                        help="Control loop frequency in Hz (default: 100.0)")

    args = parser.parse_args()

    rclpy.init()

    # mocap nodes
    right_mocap = ManusMocap(node_name='manus_mocap_right', topic_name='/manus_poses_right')
    left_mocap  = ManusMocap(node_name='manus_mocap_left',  topic_name='/manus_poses_left')

    # Load models from checkpoint tags
    print(f"[Loading] Right hand model: {args.right_ckpt}")
    right_model = load_model(args.right_ckpt)
    print(f"[Loading] Left hand model: {args.left_ckpt}")
    left_model = load_model(args.left_ckpt)

    # deployers with configurable loop_hz
    right_deployer = GeortAllegroDeployer("right", right_mocap, right_model, loop_hz=args.loop_hz)
    left_deployer  = GeortAllegroDeployer("left", left_mocap, left_model, loop_hz=args.loop_hz)

    executor = MultiThreadedExecutor()
    executor.add_node(right_mocap)
    executor.add_node(right_deployer)
    executor.add_node(left_mocap)
    executor.add_node(left_deployer)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        right_mocap.destroy_node()
        right_deployer.destroy_node()
        left_mocap.destroy_node()
        left_deployer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
