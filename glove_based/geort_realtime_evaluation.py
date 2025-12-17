#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.
import argparse
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from manus_mocap import ManusMocap
from geort import load_model, get_config
from geort.env.hand import HandKinematicModel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hand', type=str, default='allegro')
    parser.add_argument('--ckpt', type=str)
    parser.add_argument('--use_last', action='store_true',
                        help='Load last checkpoint instead of best checkpoint (default: best)')
    args = parser.parse_args()

    # ROS2 init & mocap node
    rclpy.init()
    side = 'left' if 'left' in args.hand else 'right'
    mocap = ManusMocap(topic_name=f'/manus_poses_{side}')
    executor = MultiThreadedExecutor()
    executor.add_node(mocap)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # GeoRT model & hand simulator
    epoch_to_load = 0 if args.use_last else 'best'
    model = load_model(args.ckpt, epoch=epoch_to_load)

    config = get_config(args.hand)
    hand = HandKinematicModel.build_from_config(config, render=True)
    viewer_env = hand.get_viewer_env()

    try:
        while rclpy.ok():

            for i in range(10):
                viewer_env.update()

            data = mocap.get()

            # 2) Data → qpos → set target
            points = data['result']

            t1 = time.perf_counter()

            qpos = model.forward(points)
            t2 = time.perf_counter()
            print("Model Processing Time: {:.3f}".format(t2 - t1))
            hand.set_qpos_target(qpos)
            t3 = time.perf_counter()
            print("Hand Processing Time: {:.3f}".format(t3 - t2))

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mocap.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=0.1)

if __name__ == '__main__':
    main()
