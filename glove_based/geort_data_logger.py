#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.
import time
import rclpy
from manus_mocap import ManusMocap
import numpy as np
from datetime import datetime
import geort

def main(args):
    rclpy.init()
    mocap = ManusMocap(topic_name=f'/manus_poses_{args.handness}')

    print("Starting data collection… Press Ctrl+C to stop early.")
    print("========================================")
    print("Expecting duration:", args.duration, "seconds at", args.hz, "Hz")
    print("Collecting data for hand:", args.handness)
    print("Total timeout:", args.duration / 60, "minutes")
    print("Total data points expected:", args.duration * args.hz)
    print("========================================")

    input("Press Enter to start…")

    all_results = []

    try:
        while rclpy.ok():
            # let ROS invoke callbacks for up to 100 ms
            rclpy.spin_once(mocap, timeout_sec=0.1)

            # now grab the latest frame
            data = mocap.get()
            if data['status'] == 'streaming':
                points = data['result']       # Nx3 NumPy array
                print(f"Received {points.shape[0]} points:", points)
                all_results.append(points)
                print("Data collected:", len(all_results))

            else:
                print("No data yet…")

            time.sleep(1.0 / args.hz)  # throttle your own loop

            if len(all_results) > args.duration * args.hz:
                break

        # Save!
        time_tag = datetime.now().strftime("%m%d_%H%M%S")
        save_path = geort.save_human_data(np.array(all_results), args.name + "_" + args.handness + "_"+ time_tag +".npy")
        print("Data saved to", save_path)

    except KeyboardInterrupt:
        pass
    finally:
        mocap.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='human1', type=str)  # the data package name.
    parser.add_argument('--handness', default='right', type=str)  # the data package name.
    parser.add_argument('--duration', default=60*5, type=int)  # duration in seconds. (5 minutes default)
    parser.add_argument('--hz', default=20, type=int)  # expected frame rate.
    args = parser.parse_args()

    main(args)
