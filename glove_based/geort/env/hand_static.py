#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

"""
Static URDF viewer for quick visual inspection.

Features:
- Colors each link differently for visual distinction
- Prints joint/link tree with limits
- No motion, no controllers - just a static render loop
"""

import argparse
import colorsys
import random

import numpy as np
import sapien.core as sapien
from sapien.utils import Viewer


def make_scene(enable_render=True):
    """Create engine, scene, and (optionally) renderer."""
    engine = sapien.Engine()
    renderer = None
    if enable_render:
        renderer = sapien.VulkanRenderer()
        engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)
    scene.set_ambient_light([0.6, 0.6, 0.6])
    scene.add_directional_light([0.5, 1.0, -1.0], [0.6, 0.6, 0.6], shadow=False)
    scene.add_ground(altitude=0.0)
    return engine, scene, renderer


def load_articulation(scene, urdf_path, fix_root=True):
    loader = scene.create_urdf_loader()
    loader.fix_root_link = fix_root
    art = loader.load(urdf_path)
    if art is None:
        raise FileNotFoundError(f"Failed to load URDF: {urdf_path}")
    art.set_root_pose(sapien.Pose([0, 0, 0.3]))
    return art


def random_color(alpha=1.0):
    """Generate a random color with good saturation and value."""
    h = random.random()
    s = 0.6
    v = 0.9
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return [r, g, b, alpha]


def color_links(renderer, articulation):
    """
    Color each link with a random color.

    Tries multiple APIs depending on SAPIEN version:
    - link.set_visual_material(material) (newer)
    - Fallback: iterate visual bodies > render shapes
    """
    if renderer is None:
        return
    for link in articulation.get_links():
        mat = renderer.create_material()
        mat.set_base_color(random_color(1.0))
        # Try direct API
        try:
            link.set_visual_material(mat)
            continue
        except Exception:
            pass
        # Fallback path
        try:
            vbs = link.get_visual_bodies()
        except Exception:
            vbs = []
        for vb in vbs:
            try:
                for rs in vb.get_render_shapes():
                    rs.set_material(mat)
            except Exception:
                pass


def print_tree(articulation):
    """Print articulation summary including links and joints with their limits."""
    print("\n=== Articulation Summary ===")
    print(f"Name: {articulation.get_name()}")
    links = articulation.get_links()

    # get_joints() may include fixed; get_active_joints() excludes fixed
    try:
        joints = articulation.get_joints()
        if not joints:
            joints = articulation.get_active_joints()
    except Exception:
        joints = articulation.get_active_joints()

    print(f"Links ({len(links)}):")
    for i, l in enumerate(links):
        print(f"  [{i:02d}] {l.get_name()}")

    print(f"\nJoints ({len(joints)}):")
    for j in joints:
        try:
            name = j.get_name()
        except Exception:
            name = "<unnamed>"
        try:
            parent = j.get_parent_link().get_name()
        except Exception:
            parent = "<root or N/A>"
        try:
            child = j.get_child_link().get_name()
        except Exception:
            child = "<N/A>"

        # Limits (if present)
        lim_txt = ""
        try:
            lims = j.get_limits()
            flat = []
            for pair in lims:
                if isinstance(pair, (list, tuple)) and len(pair) == 2:
                    flat.append(f"{pair[0]:.3f}..{pair[1]:.3f}")
            if flat:
                lim_txt = f"  limits: {', '.join(flat)}"
        except Exception:
            pass

        # Type (if available)
        typ_txt = ""
        try:
            typ_txt = f"  type: {j.get_joint_type()}"
        except Exception:
            pass

        print(f"  - {name}: {parent} -> {child}{typ_txt}{lim_txt}")


def place_tiny_axes(scene, pose, length=0.03, radius=0.0015):
    """
    Draw tiny RGB coordinate axes at the given pose using capsules.

    Args:
        scene: SAPIEN scene
        pose: Pose to place the axes at
        length: Length of each axis
        radius: Radius of each axis capsule
    """
    # X axis (red)
    scene.add_capsule_visual(
        pose=pose * sapien.Pose([length / 2, 0, 0], [0.7071, 0, 0.7071, 0]),
        radius=radius, half_length=length / 2, color=[1, 0, 0]
    )
    # Y axis (green)
    scene.add_capsule_visual(
        pose=pose * sapien.Pose([0, length / 2, 0], [0, 0.7071, 0.7071, 0]),
        radius=radius, half_length=length / 2, color=[0, 1, 0]
    )
    # Z axis (blue)
    scene.add_capsule_visual(
        pose=pose * sapien.Pose([0, 0, length / 2], [0, 0, 0, 1]),
        radius=radius, half_length=length / 2, color=[0, 0, 1]
    )


def main():
    """Main entry point for the static URDF viewer."""
    parser = argparse.ArgumentParser(description="Static URDF link/joint inspector with coloring")
    parser.add_argument("--handness", type=str, required=True, choices=["left", "right"],
                        help="Specify handness (left or right)")
    parser.add_argument("--no-fix-root", action="store_true", help="Do not fix root link")
    parser.add_argument("--seed", type=int, default=0, help="Color randomness seed")
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    if args.handness == "left":
        urdf_path = "./assets/allegro_left/allegro_hand_left.urdf"
    else:
        urdf_path = "./assets/allegro_right/allegro_hand_right.urdf"

    engine, scene, renderer = make_scene(enable_render=True)
    art = load_articulation(scene, urdf_path, fix_root=not args.no_fix_root)

    # Color each link
    color_links(renderer, art)

    # Console summary
    print_tree(art)

    # Optional: draw tiny axes at the base link to help orientation
    try:
        base_link = art.get_links()[0]
        base_pose = base_link.get_pose()
        place_tiny_axes(scene, base_pose)
    except Exception:
        pass

    # Viewer
    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.window.set_camera_position([0.4, -0.6, 0.5])
    viewer.window.set_camera_rotation([0.87, 0.32, 0.12, 0.34])
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1.0)

    print("\nViewer running. Close the window to exit.")
    while not viewer.closed:
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
