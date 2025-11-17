# RobotDataGenerator: Modified by GeoRTTrainer

import numpy as np
from geort.utils.config_utils import get_config
from geort.env.hand import HandKinematicModel
from geort.dataset import RobotKinematicsDataset #, MultiPointDataset
from geort.utils.path import get_data_root
from datetime import datetime
from tqdm import tqdm
import os
import math
from pathlib import Path

def merge_dict_list(dl):
    if len(dl) == 0:
        return {}
    keys = dl[0].keys()
    result = {k: [] for k in keys}
    for data in dl:
        for k in keys:
            result[k].append(data[k])
    # np.array(list_of_arrays) -> (N, 3) or (N, M, 3)
    result = {k: np.array(v, dtype=np.float32) for k, v in result.items()}
    return result


class RobotDataGenerator:
    def __init__(self, config, render=False):
        self.config = config
        self.render = render
        self.hand = HandKinematicModel.build_from_config(self.config, render=render)

    def get_robot_pointcloud(self, keypoint_names):
        kinematics_dataset = self.get_robot_kinematics_dataset()
        return kinematics_dataset.export_robot_pointcloud(keypoint_names)

    def get_robot_kinematics_dataset(self):
        dataset_path = self.get_robot_kinematics_dataset_path(postfix=True)
        if not os.path.exists(dataset_path):
            _ = self.generate_robot_kinematics_dataset(n_total=100000, save=True)
            dataset_path = self.get_robot_kinematics_dataset_path(postfix=True)
        keypoint_names = self.get_keypoint_info()["link"]
        kinematics_dataset = RobotKinematicsDataset(dataset_path, keypoint_names=keypoint_names)
        return kinematics_dataset

    def get_robot_kinematics_dataset_path(self, postfix=False):
        data_name = self.config["name"]
        out = str(Path(get_data_root()) / data_name)
        if postfix:
            out += '.npz'
        return out

    def get_keypoint_info(self):
        keypoint_links, keypoint_offsets, keypoint_joints, keypoint_human_ids = [], [], [], []
        joint_order = self.config["joint_order"]
        for info in self.config["fingertip_link"]:
            keypoint_links.append(info["link"])
            keypoint_offsets.append(info['center_offset'])
            keypoint_human_ids.append(info['human_hand_id'])
            keypoint_joints.append([joint_order.index(j) for j in info["joint"]])
        out = {"link": keypoint_links, "offset": keypoint_offsets, "joint": keypoint_joints, "human_id": keypoint_human_ids}
        print("[get_keypoint_info]", out)
        return out

    def generate_robot_kinematics_dataset(self, n_total=100000, save=True):
        """
        Generate (joint position, keypoint position) dataset.
        - Joint order: config['joint_order']
        - Keypoint order: config['fingertip_link']
        """
        info = self.get_keypoint_info()

        self.hand.initialize_keypoint(
            keypoint_link_names=info["link"],
            keypoint_offsets=info["offset"]
            )

        joint_range_low, joint_range_high = self.hand.get_joint_limit()
        joint_range_low = np.array(joint_range_low, dtype=np.float32)
        joint_range_high = np.array(joint_range_high, dtype=np.float32)

        all_data_qpos = []
        all_data_keypoint = []

        for i in tqdm(range(n_total), desc="Sampling kinematics"):

            qpos = np.random.uniform(0.0, 1.0, len(joint_range_low)).astype(np.float32) \
                   * (joint_range_high - joint_range_low) + joint_range_low
            keypoint = self.hand.keypoint_from_qpos(qpos)  # dict: link -> (K,3) or (3,)

            keypoint = {k: np.array(v, dtype=np.float32) for k, v in keypoint.items()}
            # key is the link name

            all_data_keypoint.append(keypoint)

            all_data_qpos.append(qpos.astype(np.float32))

        # stack shapes: qpos -> (N, dof); keypoint[link] -> (N, 3) or (N, Ki, 3)
        qpos_arr = np.stack(all_data_qpos, axis=0).astype(np.float32)
        keypoint_dict = merge_dict_list(all_data_keypoint)

        dataset = {"qpos": qpos_arr, "keypoint": keypoint_dict}

        if save:
            os.makedirs(get_data_root(), exist_ok=True)
            # NOTE: 'keypoint' is a dict; your RobotKinematicsDataset should load with allow_pickle=True.
            np.savez(self.get_robot_kinematics_dataset_path(), **dataset)
            print(f"[saved] {self.get_robot_kinematics_dataset_path(postfix=True)}")

        return dataset

    def visualize_samples(self, n_samples=100, sample_interval=0.1):
        """
        Visualize random hand configurations in the viewer.

        Args:
            n_samples: Number of configurations to show
            sample_interval: Time interval between samples in seconds
        """
        if not self.render:
            print("[Warning] Render is disabled. Cannot visualize samples.")
            return

        import time

        print(f"\n[Visualization Mode] Showing {n_samples} random configurations")
        print("Press Ctrl+C to stop early, or wait for all samples to display\n")

        viewer_env = self.hand.get_viewer_env()
        info = self.get_keypoint_info()

        # Initialize keypoints for visualization
        self.hand.initialize_keypoint(
            keypoint_link_names=info["link"],
            keypoint_offsets=info["offset"]
        )

        joint_range_low, joint_range_high = self.hand.get_joint_limit()
        joint_range_low = np.array(joint_range_low, dtype=np.float32)
        joint_range_high = np.array(joint_range_high, dtype=np.float32)

        try:
            for i in range(n_samples):
                # Generate random joint configuration
                qpos = np.random.uniform(0.0, 1.0, len(joint_range_low)).astype(np.float32) \
                       * (joint_range_high - joint_range_low) + joint_range_low

                # Set target joint positions
                self.hand.set_qpos_target(qpos)

                # Update simulation and render (multiple steps for smooth transition)
                for _ in range(10):
                    viewer_env.update()

                # Display progress
                if (i + 1) % 10 == 0:
                    print(f"Displayed {i + 1}/{n_samples} configurations")

                time.sleep(sample_interval)

        except KeyboardInterrupt:
            print(f"\n[Visualization stopped] Displayed {i + 1} configurations")

        print("\n[Visualization complete]")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Generate robot kinematics dataset for Allegro hand',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Generate and save dataset (default)
  %(prog)s --hand allegro_left

  # Visualize only (no dataset generation)
  %(prog)s --hand allegro_left --viz

  # Preview mode: generate small dataset and visualize
  %(prog)s --hand allegro_left --num-samples 100 --no-save

  # Generate large dataset with visualization
  %(prog)s --hand allegro_left --num-samples 100000 --viz --save
        ''')

    parser.add_argument('--hand', type=str, default='allegro_right',
                        help="hand configuration (allegro_left or allegro_right)")
    parser.add_argument('--num-samples', type=int, default=None,
                        help="number of samples (default: 1000000 for generation, 100 for viz-only)")
    parser.add_argument('--viz', action='store_true',
                        help="enable visualization mode")
    parser.add_argument('--save', action='store_true',
                        help="save dataset (use with --viz to visualize after saving)")
    parser.add_argument('--no-save', action='store_true',
                        help="don't save dataset (implies --viz for preview mode)")
    parser.add_argument('--interval', type=float, default=0.1,
                        help="visualization interval in seconds (default: 0.1)")
    args = parser.parse_args()

    # Determine operation mode
    viz_only = args.viz and not args.save and not args.no_save
    preview_mode = args.no_save or (args.viz and args.save)
    should_save = args.save or (not args.viz and not args.no_save)
    should_visualize = args.viz or args.no_save

    # Set default sample count based on mode
    if args.num_samples is None:
        args.num_samples = 100 if viz_only else 1000000

    config = get_config(args.hand)
    generator = RobotDataGenerator(config, render=should_visualize)

    if viz_only:
        # Visualization-only mode
        print(f"[Visualization Mode] Hand: {args.hand}, Samples: {args.num_samples}")
        generator.visualize_samples(n_samples=args.num_samples, sample_interval=args.interval)
    else:
        # Generate dataset
        print(f"[Generation Mode] Hand: {args.hand}, Samples: {args.num_samples}, Save: {should_save}")
        dataset = generator.generate_robot_kinematics_dataset(n_total=args.num_samples, save=should_save)

        # Show visualization if requested
        if should_visualize:
            print("\n" + "="*70)
            print("Starting visualization...")
            print("="*70 + "\n")
            viz_samples = min(100, args.num_samples)  # Visualize up to 100 samples
            generator.visualize_samples(n_samples=viz_samples, sample_interval=args.interval)
