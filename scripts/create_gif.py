#!/usr/bin/env python3
"""
Render a GIF animation from a robot_body_logger CSV.

Usage:
    python3 scripts/create_gif.py <body_trajectory.csv> <output.gif>
"""
import argparse

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def main() -> int:
    parser = argparse.ArgumentParser(description='Render GIF from body trajectory CSV.')
    parser.add_argument('source_csv', help='Path to body_trajectory.csv')
    parser.add_argument('output_gif', help='Path to output GIF file')
    args = parser.parse_args()

    df = pd.read_csv(args.source_csv)
    frames = ['motor_with_onshape_mounting'] + \
             [f'motor_with_onshape_mounting_{i}' for i in range(2, 7)] + \
             ['motor_with_no_arms']

    x_cols = [f'{f}_x' for f in frames] + ['com_x']
    y_cols = [f'{f}_y' for f in frames] + ['com_y']
    x_min, x_max = df[x_cols].min().min(), df[x_cols].max().max()
    y_min, y_max = df[y_cols].min().min(), df[y_cols].max().max()
    pad = 0.05 * max(x_max - x_min, y_max - y_min, 1e-6)

    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(x_min - pad, x_max + pad)
    ax.set_ylim(y_min - pad, y_max + pad)
    body_line, = ax.plot([], [], 'o-', color='tab:blue')
    com_trail, = ax.plot([], [], '-', color='tab:orange', lw=1)
    com_dot,   = ax.plot([], [], 'o', color='red', ms=6)

    def update(k):
        r = df.iloc[k]
        body_line.set_data([r[f'{f}_x'] for f in frames],
                           [r[f'{f}_y'] for f in frames])
        com_trail.set_data(df['com_x'].iloc[:k+1], df['com_y'].iloc[:k+1])
        com_dot.set_data([r['com_x']], [r['com_y']])
        return body_line, com_trail, com_dot

    ani = FuncAnimation(fig, update, frames=len(df), interval=50, blit=True)
    ani.save(args.output_gif, fps=20)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
