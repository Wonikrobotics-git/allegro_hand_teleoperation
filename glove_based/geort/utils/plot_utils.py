#!/usr/bin/env python3
# Copyright (c) 2025 Wonik Robotics
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os
import os
import torch
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader
from statistics import mean, stdev
import numpy as np
from pathlib import Path


# Helper utilities for loss analysis


def _flatten_gradients(grads):
    """Flatten a list of gradient tensors into a single 1-D vector.
    grads may contain None entries (allow_unused=True in autograd.grad).
    Returns a 1-D tensor (may be empty).
    """
    vecs = []
    for g in grads:
        if g is None:
            continue
        vecs.append(g.detach().view(-1))
    if len(vecs) == 0:
        return torch.tensor([], device=grads[0].device if grads else 'cpu')
    return torch.cat(vecs)

def _compute_grad_norm_and_vec(loss_term, params, retain_graph=True):
    """Compute autograd gradients of loss_term w.r.t params and return (norm, vector).
    This uses torch.autograd.grad so it won't accumulate into .grad.
    """
    grads = torch.autograd.grad(loss_term, params, retain_graph=retain_graph, allow_unused=True)
    vec = _flatten_gradients(grads)
    norm = float(vec.norm().item()) if vec.numel() > 0 else 0.0
    return norm, vec

def draw_chamfer_loss(inp_list, tgt_list, dmat_list, nn_idx_list, fig_finger_name, human_name="unknown", robot_name="unknown", RIGHT=True):
    assert len(inp_list) == len(tgt_list) == len(dmat_list) == len(nn_idx_list)
    n = len(inp_list)  # expected number of columns (should be 4)

    if n == 0:
        print("[warning] draw_chamfer_loss: empty input lists, nothing to plot.")
        return

    # Layout: 2 rows x 3 cols. Right-most column will span both rows for the big plot.
    specs = [
        [{"type": "scene"}, {"type": "scene"}, {"type": "scene", "rowspan": 2}],
        [{"type": "scene"}, {"type": "scene"}, None]
    ]

    # Titles for small 2x2 grid (left) and big plot (right)
    col_titles = fig_finger_name #  ["index", "middle", "ring", "thumb"] # json order
    # subplot_titles needs length rows*cols = 6; put None for the cell covered by rowspan
    subplot_titles = [col_titles[0], col_titles[1], "All fingers", col_titles[2], col_titles[3], None]

    figp = make_subplots(rows=2, cols=3,
                        specs=specs,
                        subplot_titles=subplot_titles,
                        column_widths=[0.25, 0.25, 0.5],
                        row_heights=[0.5, 0.5],
                        horizontal_spacing=0.06, vertical_spacing=0.06)
                        # horizontal_spacing=0.07, vertical_spacing=0.07)

    # mapping small subplot positions (index order -> subplot position)
    small_positions = [(1, 1), (1, 2), (2, 1), (2, 2)]

    # colors per finger for the big combined plot
    finger_colors = ['blue', 'green', 'magenta', 'orange']
    # darker variants for target points (minimal change)
    human_data_colors = ['darkblue', 'darkgreen', 'darkmagenta', 'darkorange']

    # add each finger's small subplot and also collect traces for the big plot
    for idx in range(n):
        inp0 = inp_list[idx][0].numpy()
        tgt0 = tgt_list[idx][0].numpy()
        # dmat and nn_idx are unused now (heatmap removed), but keep variables to avoid break
        _ = dmat_list[idx]
        nn_idx = nn_idx_list[idx]

        # small subplot coordinates
        r, c = small_positions[idx]

        # input scatter (small)
        inp_scatter = go.Scatter3d(x=inp0[:, 0], y=inp0[:, 1], z=inp0[:, 2],
                                mode='markers',
                                marker=dict(size=3, opacity=0.6, color=human_data_colors[idx]),
                                name=f'Input ({col_titles[idx]})',
                                showlegend=False)  # legend only in combined plot
        figp.add_trace(inp_scatter, row=r, col=c)

        # target scatter (small)
        tgt_scatter = go.Scatter3d(x=tgt0[:, 0], y=tgt0[:, 1], z=tgt0[:, 2],
                                mode='markers',
                                marker=dict(size=2, opacity=0.4, color=finger_colors[idx], symbol='diamond'),
                                name=f'Target ({col_titles[idx]})',
                                showlegend=False)
        figp.add_trace(tgt_scatter, row=r, col=c)

        # connections (small)
        xs, ys, zs = [], [], []
        for j, p in enumerate(inp0):
            q = tgt0[nn_idx[j]]
            xs.extend([p[0], q[0], None])
            ys.extend([p[1], q[1], None])
            zs.extend([p[2], q[2], None])
        conn_trace = go.Scatter3d(x=xs, y=ys, z=zs, mode='lines',
                                line=dict(color='gray', width=1),
                                opacity=0.9,
                                name=f'Conn ({col_titles[idx]})',
                                showlegend=False)
        figp.add_trace(conn_trace, row=r, col=c)

        # add corresponding traces to the big combined plot (right column)
        # Input points (per-finger)
        big_inp = go.Scatter3d(x=inp0[:, 0], y=inp0[:, 1], z=inp0[:, 2],
                            mode='markers',
                            marker=dict(size=3, opacity=0.6, color=human_data_colors[idx]),
                            name=f'Input {col_titles[idx]}',
                            showlegend=True)
        figp.add_trace(big_inp, row=1, col=3)

        # Target points (per-finger)
        big_tgt = go.Scatter3d(x=tgt0[:, 0], y=tgt0[:, 1], z=tgt0[:, 2],
                            mode='markers',
                            marker=dict(size=2, opacity=0.4, color=finger_colors[idx], symbol='diamond'),
                            name=f'Target {col_titles[idx]}',
                            showlegend=True)
        figp.add_trace(big_tgt, row=1, col=3)

        # Optional: connections in big plot (light, optional)
        big_xs, big_ys, big_zs = [], [], []
        for j, p in enumerate(inp0):
            q = tgt0[nn_idx[j]]
            big_xs.extend([p[0], q[0], None])
            big_ys.extend([p[1], q[1], None])
            big_zs.extend([p[2], q[2], None])
        big_conn = go.Scatter3d(x=big_xs, y=big_ys, z=big_zs, mode='lines',
                                line=dict(color='gray', width=2),
                                opacity=0.9,
                                name=f'Conn {col_titles[idx]}',
                                showlegend=True)
        figp.add_trace(big_conn, row=1, col=3)

    # set axis titles for all scene subplots (iterate over scene keys)
    for key in list(figp.layout):
        if key.startswith('scene'):
            try:
                figp.layout[key].update(xaxis_title='X', yaxis_title='Y', zaxis_title='Z')
            except Exception:
                # fallback safe update
                figp.layout[key].xaxis.title.text = 'X'

    # layout tuning: bigger size, legend on top, margins
    title_str = "Chamfer Distance — left: per-finger | right: all fingers"
    figp.update_layout(
        title=title_str,
        height=1000,
        width=1600,
        autosize=False,
        legend=dict(orientation='h', yanchor='bottom', y=1.02, x=0.5, xanchor='center'),
        margin=dict(t=120, r=40, l=20)
    )

    # Import get_data_root to save in correct location
    from geort.utils.path import get_data_root

    # Format: chamfer_{human_name}_{robot_name}.html
    filename = f"chamfer_{human_name}_{robot_name}.html"
    out_path = Path(get_data_root()) / filename

    figp.write_html(str(out_path), include_plotlyjs='cdn')

    print(f"[debug] Plotly debug HTML saved to: {out_path.as_posix()}")
