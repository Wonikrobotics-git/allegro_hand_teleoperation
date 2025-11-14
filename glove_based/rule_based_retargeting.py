#!/usr/bin/env python3
# Copyright (c) 2025 WonikRobotics_official
# Additional Contributions Copyright (c) 2025 Jungyeon Lee
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

"""
Manus Glove to Allegro Hand Teleoperation with Heuristic Retargeting

This module provides real-time teleoperation from Manus gloves to Allegro robotic hands
using a heuristic rule-based retargeting approach with Open3D visualization.
"""

import sys
import argparse
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from loop_rate_limiters import RateLimiter
from sensor_msgs.msg import JointState
from manus_ros2_msgs.msg import ManusGlove
from std_msgs.msg import Float64MultiArray

# Import common visualization utilities
try:
    from .common_viz import (
        GloveViz, ros_to_open3d_pos, ros_to_open3d_rot,
        open3d_to_allegro_pos, open3d_to_allegro_rot
    )
    from ..allegro import AllegroCommandForwarder
except ImportError:
    # Direct script execution: add parent directory to path
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    sys.path.insert(0, str(Path(__file__).parent))
    from common_viz import (
        GloveViz, ros_to_open3d_pos, ros_to_open3d_rot,
        open3d_to_allegro_pos, open3d_to_allegro_rot
    )
    from allegro import AllegroCommandForwarder


# Joint limits for Allegro hand (radians)
ALLEGRO_UPPER_LIMITS = np.array([
    1.3960,   1.1630,  1.6440,  1.7190,  # thumb
    0.47000,  1.6100,  1.7090,  1.6180,  # index
    0.47000,  1.6100,  1.7090,  1.6180,  # middle
    0.47000,  1.6100,  1.7090,  1.6180   # ring
])

ALLEGRO_LOWER_LIMITS = np.array([
     0.7000,  0.3000, -0.1890, -0.1620,  # thumb
    -0.4700, -0.1960, -0.1740, -0.2270,  # index
    -0.4700, -0.1960, -0.1740, -0.2270,  # middle
    -0.4700, -0.1960, -0.1740, -0.2270   # ring
])


class ManusAllegroHeuristicDeployer(Node):
    """
    ROS2 node for deploying Manus glove to Allegro hand teleoperation.

    Subscribes to Manus glove data, applies heuristic retargeting, and publishes
    commands to Allegro hand controllers while providing Open3D visualization.
    """

    def __init__(self, one_hand=False, side='right', two_robots=True):
        """
        Initialize the deployer node.

        Args:
            one_hand (bool): If True, control only one hand. If False, control both hands.
            side (str): Which hand to control in one-hand mode ('left' or 'right')
            two_robots (bool): If True, system has two robots. If False, single robot.
        """
        super().__init__("manus_allegro_heuristic_deployer")

        self.one_hand = one_hand
        self.side = side.lower()
        self.two_robots = two_robots
        self.glove_viz_map = {}

        # EMA smoothing coefficient (0 < alpha <= 1, higher = more current measurement)
        self.alpha = 0.2

        # Previous joint arrays for smoothing (per hand)
        self.prev_arr = {'left': None, 'right': None}

        # Initialize Allegro command forwarders based on configuration
        self._init_allegro_forwarders()

        # Initialize subscriptions
        self._init_subscriptions()

        # Timer to drive Open3D render loop at 50 Hz
        self.create_timer(0.02, self.timer_callback)

    def _init_allegro_forwarders(self):
        """Initialize Allegro command forwarders based on control mode."""
        if self.one_hand:
            # One-hand mode: control only specified hand
            mode_str = "single robot" if not self.two_robots else "two robots"
            self.get_logger().info(f"One-hand mode ({mode_str}): controlling {self.side} hand only")

            if self.side == 'left':
                self.allegro_left = AllegroCommandForwarder(
                    side='left', one_hand=True, two_robots=self.two_robots
                )
                self.allegro_right = None
            else:
                self.allegro_left = None
                self.allegro_right = AllegroCommandForwarder(
                    side='right', one_hand=True, two_robots=self.two_robots
                )
        else:
            # Both-hands mode: control both hands (requires two robots)
            self.get_logger().info("Two-hands mode: controlling both hands")
            if not self.two_robots:
                self.get_logger().warn("Two-hands mode requires two_robots=True. Forcing two_robots=True.")
                self.two_robots = True

            self.allegro_left = AllegroCommandForwarder(side='left', one_hand=False, two_robots=True)
            self.allegro_right = AllegroCommandForwarder(side='right', one_hand=False, two_robots=True)

    def _init_subscriptions(self):
        """Initialize ROS2 subscriptions to Manus glove topics."""
        if self.one_hand:
            topic = f"/manus_glove_{self.side}"
            self.create_subscription(ManusGlove, topic, self.glove_callback, 20)
        else:
            self.create_subscription(ManusGlove, "/manus_glove_left", self.glove_callback, 20)
            self.create_subscription(ManusGlove, "/manus_glove_right", self.glove_callback, 20)

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

    def _get_allegro_forwarder(self, side):
        """
        Get the appropriate Allegro command forwarder for a hand.

        Args:
            side (str): Hand side ('left' or 'right')

        Returns:
            AllegroCommandForwarder or None: The forwarder instance
        """
        return self.allegro_left if side.lower() == 'left' else self.allegro_right

    def _publish_allegro_command(self, allegro, arr, side):
        """
        Publish Allegro hand command if forwarder is available.

        Args:
            allegro (AllegroCommandForwarder): The forwarder instance
            arr (np.ndarray): Joint angles to publish
            side (str): Hand side for logging
        """
        if allegro is None:
            return

        if allegro.publisher_ is not None:
            msg = Float64MultiArray()
            msg.data = arr.tolist()
            allegro.publisher_.publish(msg)
        else:
            self.get_logger().warn(
                f"Controller not activated for {side} hand. Skipping command.",
                throttle_duration_sec=5.0
            )

    def _update_visualization(self, viz, msg):
        """
        Update Open3D visualization for a glove.

        Args:
            viz (GloveViz): The visualization object
            msg (ManusGlove): The glove message
        """
        # Clear previous data
        viz.node_positions.clear()
        viz.node_rotations.clear()

        # Determine leaf nodes (nodes without children)
        parent_ids = {n.parent_node_id for n in msg.raw_nodes}
        all_ids = {n.node_id for n in msg.raw_nodes}
        leaf_ids = all_ids - parent_ids

        # Process all nodes
        for node_idx, node in enumerate(msg.raw_nodes):
            nid = node.node_id
            p = node.pose.position

            # Transform position: ROS → Open3D → Allegro
            pos_o3d = ros_to_open3d_pos((p.x, p.y, p.z))
            pos_al = open3d_to_allegro_pos(pos_o3d)

            # Transform rotation: ROS → Open3D → Allegro
            q_ros = [
                node.pose.orientation.x,
                node.pose.orientation.y,
                node.pose.orientation.z,
                node.pose.orientation.w
            ]
            quat_o3d = ros_to_open3d_rot(q_ros)
            quat_al = open3d_to_allegro_rot(quat_o3d)

            # Update node visualization
            viz.update_node(
                node_id=nid,
                position=pos_al,
                rotation=quat_al,
                is_leaf=(nid in leaf_ids),
                node_index=node_idx
            )

        # Update skeleton connections
        connections = [
            (n.parent_node_id, n.node_id)
            for n in msg.raw_nodes
            if n.parent_node_id in viz.node_positions and n.node_id in viz.node_positions
        ]
        viz.update_skeleton(connections)

        # Update coordinate axes at each node
        viz.update_axes(axis_length=0.02)

    def glove_callback(self, msg: ManusGlove):
        """
        Callback for Manus glove messages.

        Args:
            msg (ManusGlove): Incoming glove data
        """
        # Ensure visualization exists for this glove
        if msg.glove_id not in self.glove_viz_map:
            self.glove_viz_map[msg.glove_id] = GloveViz(msg.glove_id, msg.side)
        viz = self.glove_viz_map[msg.glove_id]

        # Extract ergonomics values and transform to Allegro joint angles
        glove_vals = [e.value for e in msg.ergonomics]
        arr = self.transform_glove_to_allegro(glove_vals, msg.side)

        # Publish command to Allegro hand
        allegro = self._get_allegro_forwarder(msg.side)
        self._publish_allegro_command(allegro, arr, msg.side)

        # Update visualization
        self._update_visualization(viz, msg)

    def timer_callback(self):
        """Timer callback to drive Open3D rendering."""
        for viz in self.glove_viz_map.values():
            viz.poll_and_render()


def spin_node(one_hand=False, side='right', two_robots=True):
    """
    ROS2 spin in background thread.

    Args:
        one_hand (bool): Control mode
        side (str): Hand side for one-hand mode
        two_robots (bool): Robot setup configuration
    """
    rclpy.init()
    node = ManusAllegroHeuristicDeployer(one_hand=one_hand, side=side, two_robots=two_robots)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    """Main entry point for the application."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Manus Glove to Allegro Hand teleoperation with heuristic retargeting',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single robot (default: right hand only)
  python rule_based_retargeting.py --setup single

  # Single robot, specify left hand
  python rule_based_retargeting.py --setup single --hands left

  # Dual robots (default: both hands)
  python rule_based_retargeting.py --setup dual

  # Dual robots, control only right hand
  python rule_based_retargeting.py --setup dual --hands right

  # Dual robots, control only left hand
  python rule_based_retargeting.py --setup dual --hands left
        """
    )
    parser.add_argument(
        '--hands',
        type=str,
        choices=['left', 'right', 'both'],
        default=None,
        help='Which hand(s) to control: left, right, or both (default: right for single, both for dual)'
    )
    parser.add_argument(
        '--setup',
        type=str,
        choices=['single', 'dual'],
        default='single',
        help='Robot setup: single (one robot) or dual (two robots) (default: single)'
    )

    args = parser.parse_args()

    # Set default for --hands based on --setup if not specified
    if args.hands is None:
        args.hands = 'both' if args.setup == 'dual' else 'right'

    # Convert to internal parameters
    one_hand = (args.hands in ['left', 'right'])
    side = args.hands if one_hand else 'right'
    two_robots = (args.setup == 'dual')

    # Validate configuration
    if args.hands == 'both' and args.setup == 'single':
        print("WARNING: Cannot control both hands with single robot setup.")
        print("         Will control right hand only. Use --setup dual for both hands.")
        one_hand = True
        side = 'right'

    # Start ROS2 in background daemon thread
    threading.Thread(
        target=spin_node,
        args=(one_hand, side, two_robots),
        daemon=True
    ).start()

    # Keep main thread alive for visualization with rate limiter
    rate = RateLimiter(frequency=120.0, warn=False)
    try:
        while True:
            rate.sleep()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
