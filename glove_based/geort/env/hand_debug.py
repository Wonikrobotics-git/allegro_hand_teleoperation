#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

"""
Hand kinematic model and viewer environment for Allegro Hand visualization.

Provides:
- HandKinematicModel: Kinematic model with forward kinematics and keypoint computation
- HandViewerEnv: Multi-hand visualization environment with fingertip markers
- Collision filtering utilities for multi-hand scenes
"""

import numpy as np
import sapien.core as sapien
from sapien.utils import Viewer

from geort.utils.config_utils import get_config
from geort.utils.hand_utils import get_entity_by_name, get_active_joints, get_active_joint_indices


def set_articulation_collision_group(articulation, group_bit, ignore_bits):
    """
    Apply collision group/mask to ALL collision shapes of every link in the articulation.

    Compatible with SAPIEN builds where Link.set_collision_group(s) does not exist.
    """
    ALL = 0xFFFF
    collide_mask = ALL & (~ignore_bits)

    for link in articulation.get_links():
        # Some builds expose get_collision_shapes(); if not, skip safely
        try:
            shapes = link.get_collision_shapes()
        except AttributeError:
            shapes = []

        for shape in shapes:
            applied = False

            # Try common signatures on shape
            for args in ((group_bit, collide_mask, 0, 0),
                         (group_bit, collide_mask),
                         (group_bit,)):
                try:
                    shape.set_collision_group(*args)
                    applied = True
                    break
                except (AttributeError, TypeError):
                    try:
                        # Some versions use plural naming
                        shape.set_collision_groups(*args)
                        applied = True
                        break
                    except (AttributeError, TypeError):
                        pass

            # Optional: last-ditch attempt for filter-word style APIs (rare)
            if not applied:
                try:
                    # If these exist, they typically map to PhysX filter words
                    if hasattr(shape, "set_collision_filter_word0") and hasattr(shape, "set_collision_filter_word1"):
                        shape.set_collision_filter_word0(group_bit)
                        shape.set_collision_filter_word1(collide_mask)
                        applied = True
                except Exception:
                    pass

            # If still not applied, we just continue; not all shapes must have filters


class HandKinematicModel:
    def __init__(self,
                 scene=None,
                 render=False,
                 hand=None,
                 hand_urdf='',
                 n_hand_dof=16,
                 base_link='base_link',
                 joint_names=[],
                 kp=400.0,
                 kd=10):

        # If no shared scene passed in, create our own engine/renderer/scene
        self.engine = None
        if scene is None:
            engine = sapien.Engine()
            if render:
                renderer = sapien.VulkanRenderer()
                engine.set_renderer(renderer)
                print("Enable Render Mode.")
            else:
                renderer = None
            scene_config = sapien.SceneConfig()
            scene_config.default_dynamic_friction = 1.0
            scene_config.default_static_friction = 1.0
            scene_config.default_restitution = 0.00
            scene_config.contact_offset = 0.02
            scene_config.enable_pcm = False
            scene_config.solver_iterations = 25
            scene_config.solver_velocity_iterations = 1
            scene = engine.create_scene(scene_config)
            self.engine = engine
        # If a shared scene is provided, we assume its engine/renderer are already set up
        self.scene = scene
        self.renderer = self.scene.get_renderer() if hasattr(self.scene, "get_renderer") else None

        if hand is not None:
            self.hand = hand
        else:
            loader = scene.create_urdf_loader()
            self.hand = loader.load(hand_urdf)
            # Default pose (can be overridden in main)
            self.hand.set_root_pose(sapien.Pose([0, 0, 0.35], [0.695, 0, -0.718, 0]))

        self.pmodel = self.hand.create_pinocchio_model()

        # Base link
        self.base_link = get_entity_by_name(self.hand.get_links(), base_link)
        self.base_link_idx = self.hand.get_links().index(self.base_link)

        # DOF / joints
        self.all_joints = get_active_joints(self.hand, joint_names)
        all_limits = [joint.get_limits() for joint in self.all_joints]

        self.joint_names = joint_names
        self.user_idx_to_sim_idx = get_active_joint_indices(self.hand, joint_names)
        print("User-to-Sim Joint", self.user_idx_to_sim_idx)
        self.sim_idx_to_user_idx = [self.user_idx_to_sim_idx.index(i) for i in range(len(self.user_idx_to_sim_idx))]
        print("Sim-to-User Joint", self.sim_idx_to_user_idx)

        self.joint_lower_limit = np.array([l[0][0] for l in all_limits])
        self.joint_upper_limit = np.array([l[0][1] for l in all_limits])
        print(self.joint_lower_limit, self.joint_upper_limit)

        init_qpos = self.convert_user_order_to_sim_order((self.joint_lower_limit + self.joint_upper_limit) / 2)
        self.hand.set_qpos(init_qpos)
        self.hand.set_qvel(0.0 * init_qpos)
        self.qpos_target = init_qpos

        for i, joint in enumerate(self.all_joints):
            print(i, self.joint_names[i], joint, self.joint_lower_limit[i], self.joint_upper_limit[i])
            joint.set_drive_property(kp, kd, force_limit=10)

    def __del__(self):
        # If we created our own engine, allow cleanup; if shared, it's managed outside
        try:
            del self.engine
            del self.scene
        except Exception:
            pass

    def get_n_dof(self):
        return len(self.joint_lower_limit)

    def get_joint_limit(self):
        return self.joint_lower_limit, self.joint_upper_limit

    def initialize_keypoint(self, keypoint_link_names, keypoint_offsets):
        keypoint_links = [get_entity_by_name(self.hand.get_links(), link) for link in keypoint_link_names]
        print("Keypoint Links:", keypoint_links)
        print()

        keypoint_links_id_dict = {link_name: (self.hand.get_links().index(keypoint_links[i]), i)
                                  for i, link_name in enumerate(keypoint_link_names)}
        self.keypoint_links = keypoint_links
        self.keypoint_links_id_dict = keypoint_links_id_dict
        self.keypoint_offsets = np.array(keypoint_offsets)

    def convert_user_order_to_sim_order(self, qpos):
        return qpos[self.sim_idx_to_user_idx]

    def convert_sim_order_to_user_order(self, qpos_sim):
        """Convert simulator joint order -> user-specified joint order."""
        return qpos_sim[self.user_idx_to_sim_idx]

    def keypoint_from_qpos(self, qpos, ret_vec=False):
        """qpos in user order -> keypoints in base frame."""
        qpos = self.convert_user_order_to_sim_order(qpos)
        self.pmodel.compute_forward_kinematics(qpos)
        base_pose = self.pmodel.get_link_pose(self.base_link_idx)

        result = {}
        vec_result = []
        for m, (link_idx, i) in self.keypoint_links_id_dict.items():
            pose = self.pmodel.get_link_pose(link_idx)
            new_pose = sapien.Pose(
                p=pose.p + (pose.to_transformation_matrix()[:3, :3] @ self.keypoint_offsets[i].reshape(3, 1)).reshape(-1),
                q=pose.q,
            )
            x = (base_pose.inv() * new_pose).p  # convert to hand base frame
            vec_result.append(x)
            result[m] = x

        if ret_vec:
            return np.array(vec_result)
        return result

    @staticmethod
    def build_from_config(config, **kwargs):
        """Now accepts shared scene via kwargs['scene'] (optional)."""
        render = kwargs.get("render", False)
        urdf_path = config["urdf_path"]
        n_hand_dof = len(config["joint_order"])
        base_link = config["base_link"]
        joint_order = config["joint_order"]
        scene = kwargs.get("scene", None)

        model = HandKinematicModel(hand_urdf=urdf_path,
                                   render=render,
                                   n_hand_dof=n_hand_dof,
                                   base_link=base_link,
                                   joint_names=joint_order,
                                   scene=scene)
        return model

    # keep for backward compatibility (unused below)
    def get_viewer_env(self):
        return HandViewerEnv([self], scene=self.scene, renderer=self.renderer)
    
    def set_qpos_target(self, qpos):
        """
        Set PD controller targets for this hand.
        qpos is expected in user order.
        """
        # joint limit clip
        qpos = np.clip(qpos, self.joint_lower_limit + 1e-3,
                       self.joint_upper_limit - 1e-3)
        # convert to sim order
        qpos = self.convert_user_order_to_sim_order(qpos)
        self.qpos_target = qpos
        # apply to each drive
        for i in range(len(qpos)):
            self.all_joints[i].set_drive_target(self.qpos_target[i])


class HandViewerEnv:
    """Supports multiple hand models in ONE scene + ONE viewer. Shows fingertip dots for each model."""

    def __init__(self, models, scene, renderer=None):
        self.models = models
        self.scene = scene

        # basic lights/ground/timestep (do once)
        self.scene.set_timestep(1 / 100.0)
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
        self.scene.add_ground(altitude=0)

        # viewer
        if renderer is None:
            # Fallback if not passed (shouldn't happen when sharing)
            renderer = sapien.VulkanRenderer()
            self.scene.get_engine().set_renderer(renderer)
        self.viewer = Viewer(renderer)
        self.viewer.set_scene(self.scene)
        self.viewer.window.set_camera_position([0.1550926, -0.1623763, 0.7064089])
        self.viewer.window.set_camera_rotation([0.8716827, 0.3260138, 0.12817779, 0.3427167])
        self.viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

        # per-model fingertip spheres
        self.tip_spheres = []  # list of list: tip_spheres[m_idx] -> [actors...]
        self._create_tip_spheres()

    def _create_tip_spheres(self):
        """Create visual-only spheres for each model's keypoints (one-time)."""
        if len(self.tip_spheres) > 0:
            return

        # Color palette per model (useful for left/right distinction)
        color_palette = [
            [1.0, 0.0, 0.0],  # red
            [0.0, 0.4, 1.0],  # blue-ish
            [0.0, 0.8, 0.2],  # green
            [1.0, 0.6, 0.0],  # orange
        ]
        radius = 0.005  # 5 mm

        for mi, model in enumerate(self.models):
            # If keypoints not initialized, create empty list and continue
            if not hasattr(model, "keypoint_links"):
                self.tip_spheres.append([])
                continue

            model_color = color_palette[mi % len(color_palette)]
            per_model_spheres = []
            for _ in model.keypoint_links:
                builder = self.scene.create_actor_builder()
                builder.add_sphere_visual(radius=radius, color=model_color)
                sphere = builder.build_static()  # purely visual; no collision
                per_model_spheres.append(sphere)
            self.tip_spheres.append(per_model_spheres)

    def _update_tip_positions(self):
        """Each frame: move each model's spheres to its current keypoint positions."""
        for mi, model in enumerate(self.models):
            if not hasattr(model, "keypoint_links"):
                continue
            # lazily create if missing (e.g., keypoints inited after viewer)
            if mi >= len(self.tip_spheres) or len(self.tip_spheres[mi]) == 0:
                self._create_tip_spheres()
                if mi >= len(self.tip_spheres) or len(self.tip_spheres[mi]) == 0:
                    continue

            qpos_sim = model.hand.get_qpos()
            qpos_user = model.convert_sim_order_to_user_order(qpos_sim)
            keypoints = model.keypoint_from_qpos(qpos_user, ret_vec=True)  # (N, 3)

            for i, p in enumerate(keypoints):
                self.tip_spheres[mi][i].set_pose(sapien.Pose(p=p))

    def update(self):
        self.scene.step()
        self._update_tip_positions()
        self.scene.update_render()
        self.viewer.render()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Visualize Allegro hands")
    parser.add_argument(
        "--render-hand",
        choices=["left", "right", "both"],
        default="both",
        help="Which hand(s) to visualize in the viewer.",
    )
    args = parser.parse_args()

    # 1) Shared engine/renderer/scene
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    scene_cfg = sapien.SceneConfig()
    scene_cfg.default_dynamic_friction = 1.0
    scene_cfg.default_static_friction = 1.0
    scene_cfg.default_restitution = 0.00
    scene_cfg.contact_offset = 0.02
    scene_cfg.enable_pcm = False
    scene_cfg.solver_iterations = 25
    scene_cfg.solver_velocity_iterations = 1
    shared_scene = engine.create_scene(scene_cfg)

    # 2) Load configs (we might use only one of them)
    left_config = get_config("allegro_left")
    right_config = get_config("allegro_right")

    # 3) Conditionally build models into the SAME scene
    models = []
    left_model = right_model = None

    if args.render_hand in ("left", "both"):
        left_model = HandKinematicModel.build_from_config(left_config, scene=shared_scene, render=False)
        left_model.hand.set_root_pose(sapien.Pose([0.0, 0.0, 0.35], [0.695, 0.0, -0.718, 0.0]))
        models.append(left_model)

    if args.render_hand in ("right", "both"):
        right_model = HandKinematicModel.build_from_config(right_config, scene=shared_scene, render=False)
        right_model.hand.set_root_pose(sapien.Pose([0.0, 0.0, 0.35], [0.695, 0.0, -0.718, 0.0]))
        models.append(right_model)

    # 3.5) If both hands exist, filter collisions only between them
    if left_model is not None and right_model is not None:
        LEFT_GROUP  = 1 << 0  # 0b0001
        RIGHT_GROUP = 1 << 1  # 0b0010
        set_articulation_collision_group(left_model.hand,  LEFT_GROUP,  ignore_bits=RIGHT_GROUP)
        set_articulation_collision_group(right_model.hand, RIGHT_GROUP, ignore_bits=LEFT_GROUP)

    # 4) Initialize keypoints (dots positions) only for models we created
    if left_model is not None:
        l_links, l_offsets = [], []
        for info in left_config["fingertip_link"]:
            l_links.append(info["link"])
            l_offsets.append(info["center_offset"])
        left_model.initialize_keypoint(l_links, l_offsets)

    if right_model is not None:
        r_links, r_offsets = [], []
        for info in right_config["fingertip_link"]:
            r_links.append(info["link"])
            r_offsets.append(info["center_offset"])
        right_model.initialize_keypoint(r_links, r_offsets)

    # 5) One viewer env for whichever models are present
    viewer_env = HandViewerEnv(models, scene=shared_scene, renderer=renderer)

    # 6) Control loop for the active models
    #    Build per-model metadata so the loop stays generic.
    model_infos = []
    if left_model is not None:
        l_lower, l_upper = left_model.get_joint_limit()
        model_infos.append({"model": left_model, "lower": l_lower, "upper": l_upper})
    if right_model is not None:
        r_lower, r_upper = right_model.get_joint_limit()
        model_infos.append({"model": right_model, "lower": r_lower, "upper": r_upper})

    steps = 0
    while True:
        viewer_env.update()
        steps += 1
        if steps % 30 == 0:
            for info in model_infos:
                mdl, lower, upper = info["model"], info["lower"], info["upper"]
                dof = mdl.get_n_dof()
                targets = np.random.uniform(0, 1, dof) * (upper - lower - 1e-7) + lower + 1e-7
                mdl.set_qpos_target(targets)
