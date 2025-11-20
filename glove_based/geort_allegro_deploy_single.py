#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

import argparse
import rclpy
from rclpy.executors import MultiThreadedExecutor

from manus_mocap import ManusMocap
from geort import load_model
from geort_allegro_deploy import GeortAllegroDeployer

def main():
    parser = argparse.ArgumentParser(
        description='Deploy GeoRT model to single Allegro hand with Manus glove',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Example:
  # Deploy right hand with trained checkpoint
  %(prog)s --ckpt "human1_right_1028_150817_allegro_right_last" --side right

  # Deploy left hand with custom smoothing and mocap topic
  %(prog)s --ckpt "my_checkpoint" --side left --smoothing_alpha 0.7 --mocap_topic "/manus_poses_left"
        ''')

    parser.add_argument('--ckpt', type=str, required=True,
                        help="Checkpoint tag for the hand model")
    parser.add_argument('--side', type=str, required=True, choices=['left', 'right'],
                        help="Hand side (left or right)")
    parser.add_argument('--mocap_topic', type=str, default=None,
                        help="Manus mocap topic name (default: /manus_poses_{side})")
    parser.add_argument('--loop_hz', type=float, default=100.0,
                        help="Control loop frequency in Hz (default: 100.0)")
    parser.add_argument('--smoothing_alpha', type=float, default=0.9,
                        help='EMA smoothing alpha (0..1, 1=no smoothing, default: 0.9)')
    parser.add_argument('--use_last', action='store_true',
                        help='Load last checkpoint instead of best checkpoint (default: best)')

    args = parser.parse_args()

    # Validate alpha value
    if not (0.0 <= args.smoothing_alpha <= 1.0):
        raise ValueError("smoothing_alpha must be between 0.0 and 1.0")

    # Set default mocap topic if not provided
    if args.mocap_topic is None:
        args.mocap_topic = f'/manus_poses_{args.side}'

    rclpy.init()

    # mocap node
    mocap = ManusMocap(node_name=f'manus_mocap_{args.side}', topic_name=args.mocap_topic)

    # Load model from checkpoint tag
    epoch_to_load = 0 if args.use_last else 'best'
    print(f"[Loading] {args.side.capitalize()} hand model: {args.ckpt} (checkpoint: {'last' if args.use_last else 'best'})")
    model = load_model(args.ckpt, epoch=epoch_to_load)

    # deployer with configurable parameters (scale automatically loaded from checkpoint)
    # two_robots=False: single robot system (uses allegro_hand_position_controller without suffix)
    deployer = GeortAllegroDeployer(
        args.side, mocap, model,
        loop_hz=args.loop_hz,
        smoothing_alpha=args.smoothing_alpha,
        two_robots=False
    )

    executor = MultiThreadedExecutor()
    executor.add_node(mocap)
    executor.add_node(deployer)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mocap.destroy_node()
        deployer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
