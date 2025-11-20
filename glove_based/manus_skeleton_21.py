#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

import sys
import threading
import math
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from manus_ros2_msgs.msg import ManusGlove
from loop_rate_limiters import RateLimiter

#
# Summary
# Creates subscriptions only when /manus_glove_left or /manus_glove_right are visible from the node.
# Removes subscriptions if topics disappear (resource saving).
# Publishers (internal manus_poses_* etc.) are created once on first message arrival based on msg.side, as before.
#


def _digit_segments():
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
# 1) FLU(ROS) → RUF(Open3D)
# ──────────────────────────────────────────────────────────────
T = np.array([[ 0, -1,  0],
              [ 0,  0,  1],
              [ 1,  0,  0]])        # det = -1

def ros_to_open3d_pos(xyz):        return T @ np.asarray(xyz)

def ros_to_open3d_rot(q_ros):
    R_ros = R.from_quat(q_ros).as_matrix()
    R_o3d = T @ R_ros @ T.T        # det = +1
    return R.from_matrix(R_o3d).as_quat()

# ──────────────────────────────────────────────────────────────
# 2) RUF(Open3D) → Allegro(ROS)
# ──────────────────────────────────────────────────────────────
ry, rx = np.deg2rad(0), np.deg2rad(90)
R_Y = np.array([[ np.cos(ry), 0, np.sin(ry)],
                [          0, 1,          0],
                [-np.sin(ry), 0, np.cos(ry)]])
R_X = np.array([[1,          0,           0],
                [0, np.cos(rx), -np.sin(rx)],
                [0, np.sin(rx),  np.cos(rx)]])
R_O3D_TO_ALLEGRO = R_X @ R_Y            # det = +1

def open3d_to_allegro_pos(p_o3d):   return R_O3D_TO_ALLEGRO @ np.asarray(p_o3d)

def open3d_to_allegro_rot(q_o3d):
    R_o3d = R.from_quat(q_o3d).as_matrix()
    R_al  = R_O3D_TO_ALLEGRO @ R_o3d @ R_O3D_TO_ALLEGRO.T
    return R.from_matrix(R_al).as_quat()

def quaternion_to_euler(w, x, y, z):
    sinr = 2*(w*x + y*z)
    cosr = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr, cosr)

    sinp = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw

class GloveViz:
    def __init__(self, glove_id, side):
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window(window_name=f"{side} Glove {glove_id}")
        self.node_meshes = {}
        self.node_positions = {}   # keyed by original node_id
        self.node_rotations = {}
        self.axes_line_sets = {}
        self.skel_line_set = None
        self.label_line_sets = {}  # Index number label (LineSet)
        self.viz.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.012))

class ManusSkeleton21Node(Node):
    OMITTED = {5, 10, 15, 20}
    TARGET_TOPICS = ['/manus_glove_right', '/manus_glove_left']

    def __init__(self):
        super().__init__("manus_skeleton_21")

        self.glove_viz_map = {}

        # Dict for dynamic subscription management
        # key: topic name, value: subscription handle
        self._subs = {}
        self._sub_lock = threading.Lock()

        # Publishers are lazily created (per side: Right / Left)
        self.manus_poses_right_pub   = None
        self.manus_id_right_pub      = None
        self.manus_pos_id_right_pub  = None

        self.manus_poses_left_pub    = None
        self.manus_id_left_pub       = None
        self.manus_pos_id_left_pub   = None

        # Lock to prevent publisher creation race condition
        self._pub_lock = threading.Lock()

        # render/visual update (existing timer)
        self.create_timer(0.02, self.timer_callback)

        # Timer for topic existence detection (1 second interval)
        self.create_timer(1.0, self._topic_check_timer)

    def _ensure_publishers_for_side(self, side: str):
        """
        Create publishers once for side("Right"/"Left") if they don't exist.
        Uses lock for thread safety.
        """
        side_norm = (side or "").capitalize()
        with self._pub_lock:
            if side_norm == "Right":
                if self.manus_poses_right_pub is None:
                    self.get_logger().info("Creating publishers for Right hand topics")
                    self.manus_poses_right_pub   = self.create_publisher(PoseArray, 'manus_poses_right', 10)
                    self.manus_id_right_pub      = self.create_publisher(Int32MultiArray, 'manus_node_ids_right', 10)
                    self.manus_pos_id_right_pub  = self.create_publisher(Float32MultiArray, 'manus_positions_with_id_right', 10)
            elif side_norm == "Left":
                if self.manus_poses_left_pub is None:
                    self.get_logger().info("Creating publishers for Left hand topics")
                    self.manus_poses_left_pub    = self.create_publisher(PoseArray, 'manus_poses_left', 10)
                    self.manus_id_left_pub       = self.create_publisher(Int32MultiArray, 'manus_node_ids_left', 10)
                    self.manus_pos_id_left_pub   = self.create_publisher(Float32MultiArray, 'manus_positions_with_id_left', 10)
            else:
                self.get_logger().warning(f"Unknown side in _ensure_publishers_for_side: '{side}'")

    def _topic_check_timer(self):
        """
        Periodically checks the current node's topic list,
        creates subscriptions for topics that match TARGET_TOPICS.
        (Removes subscriptions if topics disappear)
        """
        try:
            available = {name for name, _types in self.get_topic_names_and_types()}
        except Exception as e:
            self.get_logger().warning(f"Failed to get topics: {e}")
            return

        # create subscriptions for newly available topics
        for tname in self.TARGET_TOPICS:
            if tname in available and tname not in self._subs:
                with self._sub_lock:
                    if tname not in self._subs:  # double-check
                        try:
                            sub = self.create_subscription(ManusGlove, tname, self.glove_callback, 20)
                            self._subs[tname] = sub
                            self.get_logger().info(f"Created subscription for topic: {tname}")
                        except Exception as e:
                            self.get_logger().error(f"Failed to create subscription for {tname}: {e}")

        # remove subscriptions if topic disappeared
        to_remove = []
        for tname in list(self._subs.keys()):
            if tname not in available:
                with self._sub_lock:
                    try:
                        self.destroy_subscription(self._subs[tname])
                        to_remove.append(tname)
                        self.get_logger().info(f"Destroyed subscription for missing topic: {tname}")
                    except Exception as e:
                        self.get_logger().warning(f"Failed to destroy subscription {tname}: {e}")
        for t in to_remove:
            del self._subs[t]

    def ros_to_open3d(self, p_ros):
        x_ros, y_ros, z_ros = p_ros
        return np.array([y_ros, z_ros, -x_ros])

    def open3d_to_allegro(self, p_o3d):
        ry = np.deg2rad(180)
        rx = np.deg2rad(90)
        R_y = np.array([
            [ np.cos(ry), 0, np.sin(ry)],
            [          0, 1,          0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        R_x = np.array([
            [1,          0,           0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx),  np.cos(rx)]
        ])
        R_combined = R_x.dot(R_y)
        return R_combined.dot(np.array(p_o3d))

    def glove_callback(self, msg: ManusGlove):
        kept_nodes = sorted(
            (n for n in msg.raw_nodes if n.node_id not in self.OMITTED),
            key=lambda n: n.node_id
        )

        kept_ids = [n.node_id for n in kept_nodes]
        parent_ids = {n.parent_node_id for n in kept_nodes}
        leaf_ids = set(kept_ids) - parent_ids

        if msg.glove_id not in self.glove_viz_map:
            self.glove_viz_map[msg.glove_id] = GloveViz(msg.glove_id, msg.side)

        viz = self.glove_viz_map[msg.glove_id]
        viz.node_positions.clear()
        viz.node_rotations.clear()

        for omit_n in list(self.OMITTED & viz.node_meshes.keys()):
            viz.viz.remove_geometry(viz.node_meshes[omit_n])
            del viz.node_meshes[omit_n]
            viz.axes_line_sets.pop(omit_n, None)
            if omit_n in viz.label_line_sets:
                viz.viz.remove_geometry(viz.label_line_sets[omit_n])
                del viz.label_line_sets[omit_n]

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = f"glove_{msg.glove_id}"

        new_ids     = []
        pos_with_id = []

        for new_idx, node in enumerate(kept_nodes):
            orig_id = node.node_id
            p = node.pose.position

            # Conversion: ROS → Open3D → Allegro
            pos_o3d = ros_to_open3d_pos((p.x, p.y, p.z))
            pos_al = open3d_to_allegro_pos(pos_o3d)
            viz.node_positions[orig_id] = pos_al

            # Rotation conversion: ROS → Open3D → Allegro
            q_ros = [node.pose.orientation.x,
                     node.pose.orientation.y,
                     node.pose.orientation.z,
                     node.pose.orientation.w]
            quat_o3d = ros_to_open3d_rot(q_ros)
            quat_al = open3d_to_allegro_rot(quat_o3d)
            viz.node_rotations[orig_id] = quat_al

            # PoseArray entry with Allegro values
            pose = Pose()
            pose.position.x = float(pos_al[0])
            pose.position.y = float(pos_al[1])
            pose.position.z = float(pos_al[2])
            pose.orientation.x = float(quat_al[0])
            pose.orientation.y = float(quat_al[1])
            pose.orientation.z = float(quat_al[2])
            pose.orientation.w = float(quat_al[3])
            pose_array.poses.append(pose)

            # reindexed IDs & Allegro positions
            new_ids.append(new_idx)
            pos_with_id.extend([float(new_idx), pos_al[0], pos_al[1], pos_al[2]])

            if orig_id not in viz.node_meshes:
                sph = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
                sph.compute_vertex_normals()
                col = [1,1,0] if orig_id in leaf_ids else [0.7,0.7,0.7]
                sph.paint_uniform_color(col)
                viz.node_meshes[orig_id] = sph
                viz.viz.add_geometry(sph)

            mesh = viz.node_meshes[orig_id]
            mesh.translate(-mesh.get_center(), relative=True)
            mesh.translate(viz.node_positions[orig_id], relative=False)
            viz.viz.update_geometry(mesh)

            # index number label (LineSet) update
            label_text = str(new_idx)
            anchor = viz.node_positions[orig_id]
            if orig_id not in viz.label_line_sets:
                lbl = make_digit_lines(label_text, anchor_xyz=anchor, scale=0.004, gap=0.0015)
                viz.label_line_sets[orig_id] = lbl
                viz.viz.add_geometry(lbl)
            else:
                lbl = viz.label_line_sets[orig_id]
                new_lbl = make_digit_lines(label_text, anchor_xyz=anchor, scale=0.004, gap=0.0015)
                lbl.points = new_lbl.points
                lbl.lines  = new_lbl.lines
                lbl.colors = new_lbl.colors
                viz.viz.update_geometry(lbl)

        # Create publishers if not yet created (per side)
        self._ensure_publishers_for_side(msg.side)

        # publish (None check is a safety measure since we ensured)
        if msg.side == "Right":
            if self.manus_poses_right_pub is not None:
                self.manus_poses_right_pub.publish(pose_array)
                self.manus_id_right_pub.publish(Int32MultiArray(data=new_ids))
                self.manus_pos_id_right_pub.publish(Float32MultiArray(data=pos_with_id))
        elif msg.side == "Left":
            if self.manus_poses_left_pub is not None:
                self.manus_poses_left_pub.publish(pose_array)
                self.manus_id_left_pub.publish(Int32MultiArray(data=new_ids))
                self.manus_pos_id_left_pub.publish(Float32MultiArray(data=pos_with_id))

        self._update_lines(viz, msg.raw_nodes)
        self._update_axes(viz)

    def _update_lines(self, viz, raw_nodes):
        parent_map = {n.node_id: n.parent_node_id for n in raw_nodes}
        kept = set(viz.node_positions.keys())
        pts, idxs = [], []

        for cid in kept:
            pid = parent_map[cid]
            while pid in self.OMITTED:
                pid = parent_map[pid]
            if pid in kept:
                s = len(pts)
                pts.extend([viz.node_positions[pid], viz.node_positions[cid]])
                idxs.append([s, s+1])

        if not pts:
            return

        if viz.skel_line_set is None:
            ls = o3d.geometry.LineSet(
                o3d.utility.Vector3dVector(pts),
                o3d.utility.Vector2iVector(idxs))
            ls.paint_uniform_color([0,0,0])
            viz.skel_line_set = ls
            viz.viz.add_geometry(ls)
        else:
            viz.skel_line_set.points = o3d.utility.Vector3dVector(pts)
            viz.skel_line_set.lines  = o3d.utility.Vector2iVector(idxs)
        viz.viz.update_geometry(viz.skel_line_set)

    def _update_axes(self, viz):
        L = 0.02
        for orig_id, origin in viz.node_positions.items():
            quat = viz.node_rotations[orig_id]
            Rm = R.from_quat(quat).as_matrix()
            pts, lines, cols = [], [], []
            for i in range(3):
                i0 = len(pts)
                end = origin + Rm[:,i]*L
                pts += [origin, end]
                lines.append([i0, i0+1])
                cols.append([1,0,0] if i==0 else [0,1,0] if i==1 else [0,0,1])

            if orig_id not in viz.axes_line_sets:
                ls = o3d.geometry.LineSet(
                    o3d.utility.Vector3dVector(pts),
                    o3d.utility.Vector2iVector(lines))
                ls.colors = o3d.utility.Vector3dVector(cols)
                viz.axes_line_sets[orig_id] = ls
                viz.viz.add_geometry(ls)
            else:
                ls = viz.axes_line_sets[orig_id]
                ls.points = o3d.utility.Vector3dVector(pts)
                ls.lines  = o3d.utility.Vector2iVector(lines)
                ls.colors = o3d.utility.Vector3dVector(cols)
            viz.viz.update_geometry(ls)

    def timer_callback(self):
        for viz in self.glove_viz_map.values():
            viz.viz.poll_events()
            viz.viz.update_renderer()

def spin_node():
    rclpy.init(args=sys.argv)
    node = ManusSkeleton21Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    threading.Thread(target=spin_node, daemon=True).start()
    rate = RateLimiter(frequency=120.0, warn=False)
    while True:
        rate.sleep()

if __name__ == "__main__":
    main()
