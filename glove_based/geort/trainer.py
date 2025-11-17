"""
GeoRT Trainer - Geometric Retargeting for Robot Hands

This module implements the training pipeline for learning-based hand retargeting
from human motion capture data to robot hand configurations.
"""

# ============================================================================
# IMPORTS
# ============================================================================

# Standard library
import os
import time
import math
from pathlib import Path
from datetime import datetime
from statistics import mean, stdev

# Third-party libraries
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader
from tqdm import tqdm
import wandb
import sapien.core as sapien

# GeoRT modules
from geort.dataset import RobotKinematicsDataset, MultiPointDataset
from geort.env.hand import HandKinematicModel
from geort.formatter import HandFormatter
from geort.loss import chamfer_distance
from geort.model import FKModel, IKModel
from geort.utils.config_utils import get_config, save_json
from geort.utils.hand_utils import get_active_joints, get_active_joint_indices, get_entity_by_name
from geort.utils.path import get_checkpoint_root, get_data_root, get_human_data
from geort.utils.plot_utils import _compute_grad_norm_and_vec, draw_chamfer_loss


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def merge_dict_list(dl):
    """Merge list of dictionaries into a single dictionary with numpy arrays."""
    keys = dl[0].keys()
    result = {k: [] for k in keys}
    for data in dl:
        for k in keys:
            result[k].append(data[k])
    result = {k: np.array(v) for k, v in result.items()}
    return result


def format_loss(value):
    """Format loss value with appropriate precision."""
    return f"{value:.4e}" if math.fabs(value) < 1e-3 else f"{value:.4f}"


def get_float_list_from_np(np_vector):
    """Convert numpy vector to list of floats."""
    return [float(x) for x in np_vector.tolist()]


def generate_current_timestring():
    """Generate current timestamp string in format 'YYYY-MM-DD_HH-MM-SS'."""
    return datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


# ============================================================================
# GEORT TRAINER CLASS
# ============================================================================

class GeoRTTrainer:
    """
    GeoRT Trainer for learning geometric retargeting from human to robot hands.

    This trainer handles:
    - Robot kinematics dataset generation
    - Forward kinematics (FK) neural network training
    - Inverse kinematics (IK) neural network training with geometric losses
    """

    def __init__(self, config, wandb=None):
        """
        Initialize GeoRT trainer.

        Args:
            config (dict): Hand configuration (joint limits, keypoints, etc.)
            wandb: Weights & Biases logger instance (optional)
        """
        self.config = config
        self.hand = HandKinematicModel.build_from_config(self.config)
        self.logger = wandb

    # ------------------------------------------------------------------------
    # Robot Data Management
    # ------------------------------------------------------------------------

    def get_robot_pointcloud(self, keypoint_names):
        """Get robot fingertip point cloud for the given keypoint names."""
        kinematics_dataset = self.get_robot_kinematics_dataset()
        return kinematics_dataset.export_robot_pointcloud(keypoint_names)

    def get_robot_kinematics_dataset(self):
        """
        Get or generate robot kinematics dataset.

        Returns:
            RobotKinematicsDataset: Dataset containing joint positions and keypoint positions
        """
        dataset_path = self.get_robot_kinematics_dataset_path(postfix=True)
        if not os.path.exists(dataset_path):
            kinematics_dataset = self.generate_robot_kinematics_dataset(n_total=100000, save=True)
            return kinematics_dataset

        keypoint_names = self.get_keypoint_info()["link"]
        kinematics_dataset = RobotKinematicsDataset(dataset_path, keypoint_names=keypoint_names)
        return kinematics_dataset

    def get_robot_kinematics_dataset_path(self, postfix=False):
        """Get path to robot kinematics dataset file."""
        data_name = self.config["name"]
        out = str(Path(get_data_root()) / data_name)
        if postfix:
            out += '.npz'
        return out

    def get_keypoint_info(self):
        """
        Extract keypoint information from configuration.

        Returns:
            dict: Contains 'finger_name', 'link', 'offset', 'joint', 'human_id'
        """
        finger_names = []
        keypoint_links = []
        keypoint_offsets = []
        keypoint_joints = []
        keypoint_human_ids = []

        joint_order = self.config["joint_order"]

        for info in self.config["fingertip_link"]:
            finger_names.append(info["name"])
            keypoint_links.append(info["link"])
            keypoint_offsets.append(info['center_offset'])
            keypoint_human_ids.append(info['human_hand_id'])

            keypoint_joint = []
            for joint in info["joint"]:
                keypoint_joint.append(joint_order.index(joint))

            keypoint_joints.append(keypoint_joint)

        return {
            "finger_name": finger_names,
            "link": keypoint_links,
            "offset": keypoint_offsets,
            "joint": keypoint_joints,
            "human_id": keypoint_human_ids,
        }

    def generate_robot_kinematics_dataset(self, n_total=100000, save=True):
        """
        Generate random robot kinematics dataset.

        Args:
            n_total (int): Number of random samples to generate
            save (bool): Whether to save dataset to disk

        Returns:
            dict: Dataset with 'qpos' and 'keypoint' keys
        """
        info = self.get_keypoint_info()

        self.hand.initialize_keypoint(
            keypoint_link_names=info["link"],
            keypoint_offsets=info["offset"]
        )

        data = []
        joint_range_low, joint_range_high = self.hand.get_joint_limit()
        joint_range_low = np.array(joint_range_low)
        joint_range_high = np.array(joint_range_high)

        all_data_qpos = []
        all_data_keypoint = []

        for _ in tqdm(range(n_total)):
            qpos = np.random.uniform(0, 1, len(joint_range_low)) * (joint_range_high - joint_range_low) + joint_range_low
            keypoint = self.hand.keypoint_from_qpos(qpos)
            all_data_qpos.append(qpos)
            all_data_keypoint.append(keypoint)

        all_data_keypoint = merge_dict_list(all_data_keypoint)
        dataset = {"qpos": all_data_qpos, "keypoint": all_data_keypoint}

        if save:
            os.makedirs(get_data_root(), exist_ok=True)
            np.savez(self.get_robot_kinematics_dataset_path(), **dataset)

        return dataset

    # ------------------------------------------------------------------------
    # Forward Kinematics (FK) Model
    # ------------------------------------------------------------------------

    def get_fk_checkpoint_path(self):
        """Get path to FK model checkpoint."""
        name = self.config["name"]
        os.makedirs(get_checkpoint_root(), exist_ok=True)
        return str(Path(get_checkpoint_root()) / f"fk_model_{name}.pth")

    def get_robot_neural_fk_model(self, force_train=False):
        """
        Get or train forward kinematics neural network model.

        Args:
            force_train (bool): Force retraining even if checkpoint exists

        Returns:
            FKModel: Trained forward kinematics model
        """
        # Normalizer
        joint_lower_limit, joint_upper_limit = self.hand.get_joint_limit()
        qpos_normalizer = HandFormatter(joint_lower_limit, joint_upper_limit)

        # Model
        fk_model = FKModel(keypoint_joints=self.get_keypoint_info()["joint"]).cuda()

        # Load existing model or train new one
        fk_checkpoint_path = self.get_fk_checkpoint_path()
        if os.path.exists(fk_checkpoint_path) and not force_train:
            fk_model.load_state_dict(torch.load(fk_checkpoint_path))
        else:
            print("Train Neural Forward Kinematics (FK) from Scratch")

            fk_dataset = self.get_robot_kinematics_dataset()
            fk_dataloader = DataLoader(fk_dataset, batch_size=512, shuffle=True)
            fk_optim = optim.Adam(fk_model.parameters(), lr=5e-4)

            criterion_fk = nn.MSELoss()
            for epoch in range(FK_ITER):
                all_fk_error = 0
                for batch_idx, batch in enumerate(fk_dataloader):
                    keypoint = batch["keypoint"].cuda().float()
                    qpos = batch["qpos"].cuda().float()

                    # Normalization
                    qpos = qpos_normalizer.normalize_torch(qpos)

                    predicted_keypoint = fk_model(qpos)
                    fk_optim.zero_grad()
                    loss = criterion_fk(predicted_keypoint, keypoint)
                    loss.backward()
                    fk_optim.step()

                    all_fk_error += loss.item()

                avg_fk_error = all_fk_error / (batch_idx + 1)
                print(f"Neural FK Training Epoch: {epoch}; Training Loss: {avg_fk_error}")

                if self.logger:
                    self.logger.log({
                        "FK/Loss": avg_fk_error,
                    })

            torch.save(fk_model.state_dict(), fk_checkpoint_path)

        fk_model.eval()
        return fk_model

    # ------------------------------------------------------------------------
    # Main Training Method
    # ------------------------------------------------------------------------

    def train(self, human_data_path, **kwargs):
        """
        Main training loop for inverse kinematics (IK) model.

        Trains a neural network to map human hand poses to robot joint configurations
        using multiple geometric losses: chamfer distance, direction preservation,
        curvature smoothness, and pinch detection.

        Args:
            human_data_path (Path): Path to human motion capture data
            **kwargs: Training hyperparameters and options

        Keyword Args:
            tag (str): Experiment tag for checkpoint naming
            w_chamfer (float): Weight for chamfer distance loss
            w_curvature (float): Weight for curvature smoothness loss
            w_collision (float): Weight for collision avoidance loss
            w_pinch (float): Weight for pinch detection loss
            analysis (bool): Enable loss analysis logging
            grad_log_step (int): Frequency for gradient metric computation
            compute_gradients (bool): Whether to compute per-term gradient norms
            normalize_losses (bool): Normalize losses by running std before weighting
            ema_alpha (float): Smoothing factor for moving averages
        """
        # ====================================================================
        # Setup: Configuration & Hyperparameters
        # ====================================================================

        # Analysis flags
        analysis = kwargs.get("analysis", False)
        grad_log_step = kwargs.get("grad_log_step", 200)
        compute_gradients = kwargs.get("compute_gradients", False)
        normalize_losses = kwargs.get("normalize_losses", False)
        ema_alpha = kwargs.get("ema_alpha", 0.95)

        # Models
        fk_model = self.get_robot_neural_fk_model()
        ik_model = IKModel(keypoint_joints=self.get_keypoint_info()["joint"]).cuda()
        params = list(ik_model.parameters())
        os.makedirs(get_checkpoint_root(), exist_ok=True)

        # Optimizer
        ik_optim = optim.AdamW(ik_model.parameters(), lr=1e-4)

        # Log configuration to wandb
        if self.logger:
            self.logger.config.update(kwargs)

        # Experiment setup
        exp_tag = kwargs.get("tag", "")
        hand_model_name = self.config["name"]
        self.RIGHT = True if hand_model_name.split("_")[-1] == "right" else False

        # Loss weights
        w_chamfer = kwargs.get("w_chamfer", 80.0)
        w_curvature = kwargs.get("w_curvature", 0.1)
        w_collision = kwargs.get("w_collision", 0.0)
        w_pinch = kwargs.get("w_pinch", 1.0)

        # Extract human name from data path
        human_name = human_data_path.stem

        # ====================================================================
        # Checkpoint Directories
        # ====================================================================

        time_stamp = generate_current_timestring()
        # Format: {human_name}_{robot_hand}_{timestamp}_{tag}
        save_dir = str(Path(get_checkpoint_root()) / f"{human_name}_{hand_model_name}_{time_stamp}")
        if exp_tag != '':
            save_dir += f'_{exp_tag}'
        last_save_dir = str(Path(get_checkpoint_root()) / f"{human_name}_{hand_model_name}_last")

        os.makedirs(save_dir, exist_ok=True)
        os.makedirs(last_save_dir, exist_ok=True)

        # Save configuration
        joint_lower_limit, joint_upper_limit = self.hand.get_joint_limit()

        export_config = self.config.copy()
        export_config["joint"] = {
            "lower": get_float_list_from_np(joint_lower_limit),
            "upper": get_float_list_from_np(joint_upper_limit)
        }

        export_config["hyperparams"] = {
            "MODEL_ID": time_stamp,
            "HUMAN_DATA": str(human_data_path),
            "FK_ITER": globals().get("FK_ITER"),
            "IK_ITER": globals().get("IK_ITER"),
            "HUMAN_POINT_DATASET_N": globals().get("HUMAN_POINT_DATASET_N"),
            "POINT_BATCH_SIZE": globals().get("POINT_BATCH_SIZE"),
            "W_CHAMFER": w_chamfer,
            "W_CURVATURE": w_curvature,
            "W_COLLISION": w_collision,
            "W_PINCH": w_pinch,
            "analysis": analysis,
            "compute_gradients": compute_gradients,
            "grad_log_step": grad_log_step,
            "normalize_losses": normalize_losses,
            "ema_alpha": ema_alpha
        }

        save_json(export_config, Path(save_dir) / "config.json")
        save_json(export_config, Path(last_save_dir) / "config.json")

        # ====================================================================
        # Dataset Preparation
        # ====================================================================

        # Robot data
        robot_keypoint_names = self.get_keypoint_info()['link']
        n_keypoints = len(robot_keypoint_names)
        robot_points = self.get_robot_pointcloud(robot_keypoint_names)

        # Human data
        human_finger_idxes = self.get_keypoint_info()["human_id"]

        for robot_keypoint_name, human_id in zip(robot_keypoint_names, human_finger_idxes):
            print(f"Robot Keypoint {robot_keypoint_name}: Human Id: {human_id}")

        human_points = np.load(human_data_path)
        self.human_name = human_name  # Store for later use (e.g., visualization)
        human_points = np.array([human_points[:, idx, :3] for idx in human_finger_idxes])

        point_dataset_human = MultiPointDataset.from_points(human_points, n=HUMAN_POINT_DATASET_N)
        point_dataloader = DataLoader(point_dataset_human, batch_size=POINT_BATCH_SIZE, shuffle=True)

        # Loss normalization stats
        loss_keys = ['direction', 'chamfer', 'curvature', 'pinch']
        loss_stats = {k: {"m": 0.0, "ms": 0.0, "initialized": False} for k in loss_keys} if normalize_losses else None
        eps = 1e-6

        # ====================================================================
        # Training Loop
        # ====================================================================

        for epoch in range(IK_ITER):

            # Initialize epoch loggers
            train_log = {
                'total_loss': 0,
                'direction_scale': 0,
                'curvature_scale': 0,
            }

            analysis_log = {}
            for k in loss_keys:
                analysis_log[f"Loss/Raw/{k}"] = 0.0
                analysis_log[f"Loss/Weighted/{k}"] = 0.0
                analysis_log[f"GradNorm/{k}"] = 0.0
                if normalize_losses:
                    analysis_log[f"Loss/Normalized/{k}"] = 0.0

            for i in range(len(loss_keys)):
                for j in range(i + 1, len(loss_keys)):
                    k1 = loss_keys[i]
                    k2 = loss_keys[j]
                    analysis_log[f"GradCos/{k1}_vs_{k2}"] = 0.0

            # ----------------------------------------------------------------
            # Batch Processing
            # ----------------------------------------------------------------

            for batch_idx, batch in enumerate(point_dataloader):
                point = batch.cuda()
                joint = ik_model(point)
                embedded_point = fk_model(joint)

                # Pinch Loss
                n_finger = point.size(1)
                pinch_loss = torch.tensor(0.0, device=point.device)

                for i in range(n_finger):
                    for j in range(i + 1, n_finger):
                        distance = point[:, i, ...] - point[:, j, ...]
                        mask = (torch.norm(distance, dim=-1) < 0.015).float()
                        e_distance = ((embedded_point[:, i, ...] - embedded_point[:, j, ...]) ** 2).sum(dim=-1)
                        pinch_loss += (mask * e_distance).mean() / (mask.sum() + 1e-7) * point.size(0)

                # Curvature Loss (flatness)
                direction = F.normalize(torch.randn_like(point), dim=-1, p=2)
                curvature_scale = 0.002
                delta1 = direction * curvature_scale
                point_delta_1p = point + delta1
                point_delta_1n = point - delta1

                embedded_point_p = fk_model(ik_model(point_delta_1p))
                embedded_point_n = fk_model(ik_model(point_delta_1n))
                curvature_loss = ((embedded_point_p + embedded_point_n - 2 * embedded_point) ** 2).mean()

                # Chamfer Loss
                selected_idx = np.random.randint(0, robot_points.shape[1], 2048)
                target = torch.from_numpy(robot_points[:, selected_idx, :]).permute(1, 0, 2).float().cuda()

                if epoch == 0 and batch_idx == 0 and DRAW_CHAMFER:
                    inp_orig_list = []
                    tgt_orig_list = []
                    dmat0_list = []
                    nn_idx_list = []

                chamfer_loss = torch.tensor(0.0, device=point.device)
                for i in range(n_keypoints):
                    chamfer_loss += chamfer_distance(
                        embedded_point[:, i, :].unsqueeze(0),
                        target[:, i, :].unsqueeze(0),
                    )

                    if DRAW_CHAMFER and epoch == 0 and batch_idx == 0:
                        fig_finger_name = self.get_keypoint_info()['finger_name']
                        # print(f"FINGER IDX: {i} | FINGER NAME: {fig_finger_name[i]}")

                        inp_orig = embedded_point[:, i, :].unsqueeze(0).detach().cpu()
                        tgt_orig = target[:, i, :].unsqueeze(0).detach().cpu()
                        inp_orig_list.append(inp_orig)
                        tgt_orig_list.append(tgt_orig)

                        input_points = embedded_point[:, i, :].unsqueeze(0).clone()
                        target_points = target[:, i, :].unsqueeze(0).clone()

                        input_points = input_points.unsqueeze(2)
                        target_points = target_points.unsqueeze(1)

                        _, N, _, _ = input_points.size()
                        _, _, M, _ = target_points.size()
                        input_points_repeat = input_points.repeat(1, 1, M, 1)
                        target_points_repeat = target_points.repeat(1, N, 1, 1)

                        dist_matrix = torch.sum((input_points_repeat - target_points_repeat)**2, dim=-1)
                        dmat0 = dist_matrix[0].detach().cpu().numpy()
                        nn_idx = dmat0.argmin(axis=1)
                        dmat0_list.append(dmat0)
                        nn_idx_list.append(nn_idx)

                if DRAW_CHAMFER and epoch == 0 and batch_idx == 0:
                    draw_chamfer_loss(inp_orig_list, tgt_orig_list, dmat0_list, nn_idx_list,
                                    fig_finger_name, self.human_name, hand_model_name, self.RIGHT)

                # Direction Loss
                direction = F.normalize(torch.randn_like(point), dim=-1, p=2)
                direction_scale = 0.001 + torch.rand(point.size(0)).cuda().unsqueeze(-1).unsqueeze(-1) * 0.01
                point_delta = point + direction * direction_scale
                joint_delta = ik_model(point_delta)
                embedded_point_delta = fk_model(joint_delta)

                d1 = (point_delta - point).reshape(-1, 3)
                d2 = (embedded_point_delta - embedded_point).reshape(-1, 3)
                direction_loss = -(((F.normalize(d1, dim=-1, p=2, eps=1e-5) *
                                    F.normalize(d2, dim=-1, p=2, eps=1e-5)).sum(-1))).mean()

                # Collision Loss (placeholder)
                collision_loss = torch.tensor([0.0]).cuda().squeeze()

                # --------------------------------------------------------
                # Compose Weighted Loss
                # --------------------------------------------------------

                weighted_losses = {
                    'direction': (1.0, direction_loss),
                    'chamfer': (w_chamfer, chamfer_loss),
                    'curvature': (w_curvature, curvature_loss),
                    'pinch': (w_pinch, pinch_loss),
                }

                # Optional: Normalize losses
                normalized_losses = {}
                if normalize_losses:
                    for k, (w, t) in weighted_losses.items():
                        t_val = float(t.detach().cpu().item())
                        stats = loss_stats[k]
                        if not stats["initialized"]:
                            stats["m"] = t_val
                            stats["ms"] = t_val * t_val
                            stats["initialized"] = True
                        else:
                            stats["m"] = ema_alpha * stats["m"] + (1.0 - ema_alpha) * t_val
                            stats["ms"] = ema_alpha * stats["ms"] + (1.0 - ema_alpha) * (t_val * t_val)
                        var = max(0.0, stats["ms"] - (stats["m"] ** 2))
                        std = math.sqrt(var) if var > 0 else 0.0
                        denom = std + eps
                        t_norm = t / denom
                        normalized_losses[k] = (w, t_norm)
                        analysis_log[f"Loss/Normalized/{k}"] += float(t_norm.detach().cpu().item())
                    used_losses = normalized_losses
                else:
                    used_losses = weighted_losses

                # Build final loss
                loss = sum(w * t for w, t in [(v[0], v[1]) for v in used_losses.values()])

                # --------------------------------------------------------
                # Analysis (optional)
                # --------------------------------------------------------

                if analysis:
                    for k, (w, t) in weighted_losses.items():
                        analysis_log[f"Loss/Raw/{k}"] += float(t.detach().cpu().item())
                        analysis_log[f"Loss/Weighted/{k}"] += float((w * t).detach().cpu().item())

                # Gradient analysis
                grad_norms = {}
                grad_vecs = {}
                do_grad_compute = analysis and compute_gradients and (batch_idx % grad_log_step == 0)
                if do_grad_compute:
                    for k, (w, t) in used_losses.items():
                        w_t = (w * t)
                        gnorm, gvec = _compute_grad_norm_and_vec(w_t, params, retain_graph=True)
                        grad_norms[k] = gnorm
                        grad_vecs[k] = gvec
                        analysis_log[f"GradNorm/{k}"] += gnorm

                    keys = list(grad_vecs.keys())
                    for i in range(len(keys)):
                        for j in range(i + 1, len(keys)):
                            a = grad_vecs[keys[i]]
                            b = grad_vecs[keys[j]]
                            if a.numel() == 0 or b.numel() == 0:
                                cos = 0.0
                            else:
                                cos = float(torch.nn.functional.cosine_similarity(
                                    a.unsqueeze(0), b.unsqueeze(0)).item())
                            analysis_log[f"GradCos/{keys[i]}_vs_{keys[j]}"] += cos

                # --------------------------------------------------------
                # Backward Pass & Optimization
                # --------------------------------------------------------

                ik_optim.zero_grad()
                loss.backward()
                ik_optim.step()

                # Accumulate metrics
                train_log["total_loss"] += float(loss.detach().cpu().item())
                train_log["direction_scale"] += torch.mean(direction_scale).item() if isinstance(
                    direction_scale, torch.Tensor) else float(direction_scale)
                train_log["curvature_scale"] += curvature_scale

            # ----------------------------------------------------------------
            # End of Epoch: Logging & Checkpointing
            # ----------------------------------------------------------------

            num_batches = batch_idx + 1
            train_log = {k: v / num_batches for k, v in train_log.items()}
            analysis_log = {k: v / num_batches for k, v in analysis_log.items()}

            # Wandb logging
            if self.logger:
                logger_dict = {}
                logger_dict.update(train_log)
                logger_dict.update(analysis_log)
                self.logger.log(logger_dict)

            # Console logging
            print(
                f"Epoch {epoch} | Losses ||"
                f" Total Loss {format_loss(float(loss.detach().cpu().item()))} ||"
                f" - Direction: {format_loss(float(direction_loss.detach().cpu().item()))}"
                f" - Chamfer: {format_loss(float(chamfer_loss.detach().cpu().item()))}"
                f" - Curvature: {format_loss(float(curvature_loss.detach().cpu().item()))}"
                f" - Pinch: {format_loss(float(pinch_loss.detach().cpu().item()))}"
            )

            # Save checkpoints every 100 epochs
            if epoch % 100 == 0:
                torch.save(ik_model.state_dict(), Path(save_dir) / f"epoch_{epoch}.pth")
                torch.save(ik_model.state_dict(), Path(save_dir) / f"last.pth")
                torch.save(ik_model.state_dict(), Path(last_save_dir) / f"last.pth")

        return


# ============================================================================
# MAIN EXECUTION
# ============================================================================

if __name__ == '__main__':

    start_time = time.time()

    # ------------------------------------------------------------------------
    # Command-Line Arguments
    # ------------------------------------------------------------------------

    import argparse
    parser = argparse.ArgumentParser(description='Train GeoRT retargeting model')

    # Required arguments
    parser.add_argument('--hand', type=str, default='allegro',
                       help='Hand configuration name')
    parser.add_argument('--human_data', type=str, default='human',
                       help='Human mocap data filename')
    parser.add_argument('--ckpt_tag', type=str, default='',
                       help='Checkpoint tag for experiment naming')

    # Loss weights
    parser.add_argument('--w_chamfer', type=float, default=80.0,
                       help='Chamfer distance loss weight')
    parser.add_argument('--w_curvature', type=float, default=0.1,
                       help='Curvature smoothness loss weight')
    parser.add_argument('--w_collision', type=float, default=0.0,
                       help='Collision avoidance loss weight')
    parser.add_argument('--w_pinch', type=float, default=1.0,
                       help='Pinch detection loss weight')

    # Wandb configuration
    parser.add_argument('--wandb_project', type=str, default='geort',
                       help='Wandb project name')
    parser.add_argument('--wandb_entity', type=str, default=None,
                       help='Wandb entity (username or team name)')
    parser.add_argument('--no_wandb', action='store_true',
                       help='Disable wandb logging')

    args = parser.parse_args()

    # ------------------------------------------------------------------------
    # Hyperparameters
    # ------------------------------------------------------------------------

    DRAW_CHAMFER = True
    WANDB = not args.no_wandb
    FK_ITER = 200
    IK_ITER = 2000
    HUMAN_POINT_DATASET_N = 20000
    POINT_BATCH_SIZE = 4096
    W_CHAMFER = args.w_chamfer
    W_CURVATURE = args.w_curvature
    W_COLLISION = args.w_collision
    W_PINCH = args.w_pinch
    HUMAN_DATA_PATH = args.human_data
    ROBOT_HAND = args.hand

    # ------------------------------------------------------------------------
    # Prepare Data Paths
    # ------------------------------------------------------------------------

    config = get_config(args.hand)
    human_data_path = get_human_data(args.human_data)
    human_name = human_data_path.stem  # Extract human name for naming

    # ------------------------------------------------------------------------
    # Wandb Initialization
    # ------------------------------------------------------------------------

    if WANDB:
        # Format: {human_name}_{robot_hand}_{timestamp}_{tag}
        run_name = f"{human_name}_{args.hand}_{generate_current_timestring()}"
        if args.ckpt_tag:
            run_name += f"_{args.ckpt_tag}"

        wandb_init_kwargs = {
            'project': args.wandb_project,
            'name': run_name,
            'config': vars(args)
        }
        if args.wandb_entity is not None:
            wandb_init_kwargs['entity'] = args.wandb_entity

        run = wandb.init(**wandb_init_kwargs)
    else:
        run = None

    # ------------------------------------------------------------------------
    # Training
    # ------------------------------------------------------------------------

    trainer = GeoRTTrainer(config, wandb=run)
    print(">>>>>>> Human Data >>>>>>>\n", human_data_path.as_posix())

    trainer.train(
        human_data_path,
        tag=args.ckpt_tag,
        w_chamfer=args.w_chamfer,
        w_curvature=args.w_curvature,
        w_collision=args.w_collision,
        w_pinch=args.w_pinch,
        # Analysis flags
        analysis=True,
        compute_gradients=False,
        grad_log_step=200,
        normalize_losses=False,
        ema_alpha=0.95
    )

    # ------------------------------------------------------------------------
    # Training Complete
    # ------------------------------------------------------------------------

    end_time = time.time()
    secs = end_time - start_time
    mins = secs / 60
    time_log = f'{mins} Mins : {secs-mins*60:.3f} Secs'
    print("=" * 70)
    print(f"Training time: {time_log}")
    print("=" * 70)
