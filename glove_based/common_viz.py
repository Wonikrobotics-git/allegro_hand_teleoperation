#!/usr/bin/env python3
# Copyright (c) 2025 WonikRobotics_official
# Additional Contributions Copyright (c) 2025 Jungyeon Lee
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

"""
Common visualization utilities for Manus Glove data with Open3D.
Based on manus_skeleton_21.py implementation.
"""
import math
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R


# ──────────────────────────────────────────────────────────────
# Coordinate Transformations
# ──────────────────────────────────────────────────────────────

# 1) FLU(ROS) → RUF(Open3D)
T = np.array([[ 0, -1,  0],
              [ 0,  0,  1],
              [ 1,  0,  0]])        # det = -1

def ros_to_open3d_pos(xyz):
    """Transform position from ROS (FLU) to Open3D (RUF) coordinates"""
    return T @ np.asarray(xyz)

def ros_to_open3d_rot(q_ros):
    """Transform rotation quaternion from ROS to Open3D coordinates"""
    R_ros = R.from_quat(q_ros).as_matrix()
    R_o3d = T @ R_ros @ T.T        # det = +1
    return R.from_matrix(R_o3d).as_quat()

# 2) RUF(Open3D) → Allegro(ROS)
ry, rx = np.deg2rad(0), np.deg2rad(90)
R_Y = np.array([[ np.cos(ry), 0, np.sin(ry)],
                [          0, 1,          0],
                [-np.sin(ry), 0, np.cos(ry)]])
R_X = np.array([[1,          0,           0],
                [0, np.cos(rx), -np.sin(rx)],
                [0, np.sin(rx),  np.cos(rx)]])
R_O3D_TO_ALLEGRO = R_X @ R_Y            # det = +1

def open3d_to_allegro_pos(p_o3d):
    """Transform position from Open3D to Allegro coordinates"""
    return R_O3D_TO_ALLEGRO @ np.asarray(p_o3d)

def open3d_to_allegro_rot(q_o3d):
    """Transform rotation quaternion from Open3D to Allegro coordinates"""
    R_o3d = R.from_quat(q_o3d).as_matrix()
    R_al  = R_O3D_TO_ALLEGRO @ R_o3d @ R_O3D_TO_ALLEGRO.T
    return R.from_matrix(R_al).as_quat()

def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    sinr = 2*(w*x + y*z)
    cosr = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr, cosr)

    sinp = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


# ──────────────────────────────────────────────────────────────
# Digital Label Generation (7-segment display style)
# ──────────────────────────────────────────────────────────────

def _digit_segments():
    """Define 7-segment display segments"""
    segs = {
        'a': [(0.1, 1.9), (0.9, 1.9)],
        'b': [(0.9, 1.9), (0.9, 1.0)],
        'c': [(0.9, 1.0), (0.9, 0.1)],
        'd': [(0.1, 0.1), (0.9, 0.1)],
        'e': [(0.1, 0.1), (0.1, 1.0)],
        'f': [(0.1, 1.0), (0.1, 1.9)],
        'g': [(0.1, 1.0), (0.9, 1.0)],
    }
    on = {
        '0': ['a','b','c','d','e','f'],
        '1': ['b','c'],
        '2': ['a','b','g','e','d'],
        '3': ['a','b','g','c','d'],
        '4': ['f','g','b','c'],
        '5': ['a','f','g','c','d'],
        '6': ['a','f','g','e','c','d'],
        '7': ['a','b','c'],
        '8': ['a','b','c','d','e','f','g'],
        '9': ['a','b','c','d','f','g'],
        '-': ['g'],
    }
    return segs, on

def make_digit_lines(text: str, anchor_xyz: np.ndarray, scale=0.004, gap=0.002):
    """
    Create 7-segment style digit labels as Open3D LineSet.

    Args:
        text: String to display (digits 0-9 and '-')
        anchor_xyz: 3D position to place the label
        scale: Size of each digit
        gap: Gap between digits

    Returns:
        o3d.geometry.LineSet: The digit label
    """
    segs, on = _digit_segments()
    pts = []
    lines = []
    cursor_x = 0.0

    def to3(p2):
        return np.array([p2[0]*scale, p2[1]*scale, 0.0])

    for ch in str(text):
        act = on.get(ch, [])
        base_idx = len(pts)
        for s in act:
            p0, p1 = segs[s]
            q0 = to3(p0) + np.array([cursor_x, 0, 0])
            q1 = to3(p1) + np.array([cursor_x, 0, 0])
            pts.extend([q0, q1])
            lines.append([base_idx, base_idx+1])
            base_idx += 2
        cursor_x += scale*1.2 + gap

    if not pts:
        pts = [np.zeros(3), np.array([scale,0,0])]
        lines = [[0,1]]

    pts = np.asarray(pts)

    # Rotation to align with Open3D coordinate system
    Ry = np.array([
        [ 0, 0, 1],
        [ 0, 1, 0],
        [-1, 0, 0]
    ])
    Rx = np.array([
        [1,          0,           0],
        [0, 0, -1],
        [0, 1,  0]
    ])
    pts = pts @ Ry.T @ Rx.T

    # Offset from anchor
    offset = anchor_xyz + np.array([0.006, 0.006, 0.006])
    pts = pts + offset

    ls = o3d.geometry.LineSet(
        o3d.utility.Vector3dVector(pts),
        o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    )
    ls.colors = o3d.utility.Vector3dVector(
        np.tile(np.array([[0.1,0.1,0.1]]), (len(lines), 1))
    )
    return ls


# ──────────────────────────────────────────────────────────────
# GloveViz Class
# ──────────────────────────────────────────────────────────────

class GloveViz:
    """
    Open3D visualization for Manus Glove data.

    Features:
    - Spheres at each node position
    - Coordinate axes at each node (RGB = XYZ)
    - Skeleton lines connecting nodes
    - Index number labels for each node
    """

    def __init__(self, glove_id, side):
        """
        Initialize Open3D visualizer for a glove.

        Args:
            glove_id: Unique identifier for the glove
            side: "Left" or "Right"
        """
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window(window_name=f"{side} Glove {glove_id}")

        self.glove_id = glove_id
        self.node_meshes = {}      # {node_id: sphere_mesh}
        self.node_positions = {}   # {node_id: np.array([x, y, z])}
        self.node_rotations = {}   # {node_id: quaternion}
        self.axes_line_sets = {}   # {node_id: LineSet for XYZ axes}
        self.label_line_sets = {}  # {node_id: LineSet for index label}
        self.skel_line_set = None  # LineSet for skeleton connections

        # Add global coordinate frame at origin
        self.viz.add_geometry(
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.012)
        )

    def update_node(self, node_id, position, rotation=None,
                   is_leaf=False, node_index=None):
        """
        Update or create a node visualization.

        Args:
            node_id: Original node ID from glove data
            position: 3D position (already transformed to Allegro coordinates)
            rotation: Quaternion rotation (already transformed to Allegro coordinates)
            is_leaf: Whether this is a leaf node (affects color)
            node_index: Re-indexed node number for label
        """
        self.node_positions[node_id] = position
        if rotation is not None:
            self.node_rotations[node_id] = rotation

        # Create or update sphere mesh
        if node_id not in self.node_meshes:
            sph = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
            sph.compute_vertex_normals()
            col = [1, 1, 0] if is_leaf else [0.7, 0.7, 0.7]
            sph.paint_uniform_color(col)
            self.node_meshes[node_id] = sph
            self.viz.add_geometry(sph)

        # Update sphere position
        mesh = self.node_meshes[node_id]
        mesh.translate(-mesh.get_center(), relative=True)
        mesh.translate(position, relative=False)
        self.viz.update_geometry(mesh)

        # Update index label
        if node_index is not None:
            self._update_label(node_id, node_index, position)

    def _update_label(self, node_id, index, position):
        """Update or create index label for a node"""
        label_text = str(index)

        if node_id not in self.label_line_sets:
            lbl = make_digit_lines(label_text, anchor_xyz=position,
                                  scale=0.004, gap=0.0015)
            self.label_line_sets[node_id] = lbl
            self.viz.add_geometry(lbl)
        else:
            lbl = self.label_line_sets[node_id]
            new_lbl = make_digit_lines(label_text, anchor_xyz=position,
                                      scale=0.004, gap=0.0015)
            lbl.points = new_lbl.points
            lbl.lines = new_lbl.lines
            lbl.colors = new_lbl.colors
            self.viz.update_geometry(lbl)

    def update_skeleton(self, connections):
        """
        Update skeleton line connections.

        Args:
            connections: List of (parent_id, child_id) tuples
        """
        pts = []
        idxs = []

        for pid, cid in connections:
            if pid in self.node_positions and cid in self.node_positions:
                s = len(pts)
                pts.extend([self.node_positions[pid], self.node_positions[cid]])
                idxs.append([s, s+1])

        if not pts:
            return

        if self.skel_line_set is None:
            ls = o3d.geometry.LineSet(
                o3d.utility.Vector3dVector(pts),
                o3d.utility.Vector2iVector(idxs)
            )
            ls.paint_uniform_color([0, 0, 0])
            self.skel_line_set = ls
            self.viz.add_geometry(ls)
        else:
            self.skel_line_set.points = o3d.utility.Vector3dVector(pts)
            self.skel_line_set.lines = o3d.utility.Vector2iVector(idxs)
            self.viz.update_geometry(self.skel_line_set)

    def update_axes(self, axis_length=0.02):
        """
        Update coordinate axes for all nodes with rotation data.

        Args:
            axis_length: Length of each axis
        """
        for node_id, origin in self.node_positions.items():
            if node_id not in self.node_rotations:
                continue

            quat = self.node_rotations[node_id]
            Rm = R.from_quat(quat).as_matrix()

            pts, lines, cols = [], [], []
            for i in range(3):
                i0 = len(pts)
                end = origin + Rm[:, i] * axis_length
                pts += [origin, end]
                lines.append([i0, i0+1])
                # RGB = XYZ
                cols.append([1,0,0] if i==0 else [0,1,0] if i==1 else [0,0,1])

            if node_id not in self.axes_line_sets:
                ls = o3d.geometry.LineSet(
                    o3d.utility.Vector3dVector(pts),
                    o3d.utility.Vector2iVector(lines)
                )
                ls.colors = o3d.utility.Vector3dVector(cols)
                self.axes_line_sets[node_id] = ls
                self.viz.add_geometry(ls)
            else:
                ls = self.axes_line_sets[node_id]
                ls.points = o3d.utility.Vector3dVector(pts)
                ls.lines = o3d.utility.Vector2iVector(lines)
                ls.colors = o3d.utility.Vector3dVector(cols)
                self.viz.update_geometry(ls)

    def remove_node(self, node_id):
        """Remove a node and all its visualizations"""
        if node_id in self.node_meshes:
            self.viz.remove_geometry(self.node_meshes[node_id])
            del self.node_meshes[node_id]

        if node_id in self.axes_line_sets:
            self.viz.remove_geometry(self.axes_line_sets[node_id])
            del self.axes_line_sets[node_id]

        if node_id in self.label_line_sets:
            self.viz.remove_geometry(self.label_line_sets[node_id])
            del self.label_line_sets[node_id]

        self.node_positions.pop(node_id, None)
        self.node_rotations.pop(node_id, None)

    def poll_and_render(self):
        """Update Open3D window (call this in a timer loop)"""
        self.viz.poll_events()
        self.viz.update_renderer()
