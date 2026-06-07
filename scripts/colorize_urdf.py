#!/usr/bin/env python3
"""
Generate a recolored copy of the snake URDF whose body segments are tinted
differently, for more legible RViz screenshots (e.g. thesis gait figures).

The robot is a chain of body segments connected by `revolute` joints; each
segment also carries two `prismatic` adhesive-pad links. This script walks the
joint tree to order the body segments head→tail, assigns each one a color, and
rewrites every <visual>/<material>/<color> within that segment (and its pads).
Pads get a darker shade of their segment's color so they read as part of it.

Only material colors change — geometry, joints, and inertia are untouched — and
since Gazebo physics uses the separate SDF, this affects RViz visualization only.

Usage:
  python3 scripts/colorize_urdf.py                       # gradient → urdf/snake_colored.urdf
  python3 scripts/colorize_urdf.py --scheme palette
  python3 scripts/colorize_urdf.py --scheme alternating
  python3 scripts/colorize_urdf.py -i urdf/snake.urdf -o urdf/snake_colored.urdf
"""
import argparse
import colorsys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# Distinct categorical palette (used by --scheme palette/alternating). RGB 0..1.
PALETTE = [
    (0.90, 0.30, 0.24),   # red
    (0.95, 0.61, 0.07),   # orange
    (0.95, 0.90, 0.18),   # yellow
    (0.30, 0.76, 0.34),   # green
    (0.20, 0.60, 0.86),   # blue
    (0.50, 0.35, 0.80),   # violet
    (0.90, 0.40, 0.70),   # pink
]


def build_segment_order(root):
    """Return body links ordered head→tail and a link→segment-index map.

    Body links are connected by `revolute` joints; pad links hang off body
    links via `prismatic` joints and inherit their parent body segment's index.
    """
    joints = root.findall('joint')
    parent_of = {}          # child link -> (parent link, joint type)
    revolute_child = {}     # parent link -> child link (revolute only)
    for j in joints:
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        jtype = j.get('type')
        parent_of[child] = (parent, jtype)
        if jtype == 'revolute':
            revolute_child[parent] = child

    all_links = [l.get('name') for l in root.findall('link')]
    # Head = a body link that is never the child of a revolute joint.
    revolute_children = set(revolute_child.values())
    revolute_parents = set(revolute_child.keys())
    body_links_set = revolute_parents | revolute_children
    head = next(l for l in body_links_set if l not in revolute_children)

    # Walk the revolute chain head→tail.
    body_order = [head]
    while body_order[-1] in revolute_child:
        body_order.append(revolute_child[body_order[-1]])

    seg_index = {name: i for i, name in enumerate(body_order)}

    # Assign each remaining (pad) link the segment index of its parent.
    for link in all_links:
        if link in seg_index:
            continue
        parent = link
        # Walk up until we hit a body link (pads attach directly, but be safe).
        while parent in parent_of and parent not in seg_index:
            parent = parent_of[parent][0]
        seg_index[link] = seg_index.get(parent, 0)

    return body_order, seg_index


def segment_colors(n, scheme):
    if scheme == 'gradient':
        # Hue sweep red→violet (stop before wrapping back to red).
        return [colorsys.hsv_to_rgb(0.83 * i / max(n - 1, 1), 0.85, 0.95)
                for i in range(n)]
    if scheme == 'alternating':
        return [PALETTE[(i % 2)+3] for i in range(n)]
    # palette
    return [PALETTE[i % len(PALETTE)] for i in range(n)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-i', '--input', default=str(REPO_ROOT / 'urdf' / 'snake.urdf'))
    ap.add_argument('-o', '--output', default=str(REPO_ROOT / 'urdf' / 'snake_colored.urdf'))
    ap.add_argument('--scheme', choices=['gradient', 'palette', 'alternating'],
                    default='gradient')
    ap.add_argument('--pad-shade', type=float, default=0.55,
                    help='Brightness multiplier for adhesive-pad links (0..1).')
    args = ap.parse_args()

    tree = ET.parse(args.input)
    root = tree.getroot()

    body_order, seg_index = build_segment_order(root)
    colors = segment_colors(len(body_order), args.scheme)

    pad_links = {l.get('name') for l in root.findall('link')} - set(body_order)

    recolored = 0
    for link in root.findall('link'):
        name = link.get('name')
        r, g, b = colors[seg_index[name]]
        if name in pad_links:
            r, g, b = (c * args.pad_shade for c in (r, g, b))
        for color in link.findall('.//material/color'):
            rgba = color.get('rgba').split()
            alpha = rgba[3] if len(rgba) == 4 else '1'
            color.set('rgba', f'{r:.4f} {g:.4f} {b:.4f} {alpha}')
            recolored += 1

    tree.write(args.output, xml_declaration=True, encoding='utf-8')

    print(f'Body segments (head→tail): {len(body_order)}')
    for i, link in enumerate(body_order):
        r, g, b = colors[i]
        print(f'  [{i}] {link:35s} rgb=({r:.2f}, {g:.2f}, {b:.2f})')
    print(f'Recolored {recolored} <color> tags across '
          f'{len(body_order) + len(pad_links)} links.')
    print(f'Wrote {args.output}')


if __name__ == '__main__':
    main()
