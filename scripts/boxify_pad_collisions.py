#!/usr/bin/env python3
"""
Replace adhesive-pad mesh collisions in the snake SDF with box primitives.

Mesh-vs-plane contact with the ~450-triangle pad meshes dominates Gazebo's
per-step cost. A box primitive with the same AABB produces ≤4 contact
points per pad on the ground, cutting contact-resolution cost massively
without changing the observable contact footprint.

Reads the box dimensions from the actual STL files (AABB size + centroid),
so it stays correct if the pad geometry is regenerated with a different
shape. Preserves each collision's <surface>/<friction> block verbatim —
the friction coefficients (mu=0.001 / mu=0.99) define the locomotion
mechanism and must not be touched.

Assumes pad collision poses have RPY ≈ 0 (verified for the current SDF).
If any pose has a meaningful rotation, the script aborts rather than silently
produce a wrong offset.

Usage:
    python3 scripts/boxify_pad_collisions.py sdf/snake/snake.sdf
    python3 scripts/boxify_pad_collisions.py sdf/snake/snake.sdf --dry-run
"""
import argparse
import re
import shutil
import struct
import sys
from pathlib import Path


ROTATION_TOLERANCE = 1e-6  # radians; above this, pose rotation is non-trivial

POSE_RE = re.compile(r'(<pose[^>]*>)([^<]*)(</pose>)')
COLLISION_OPEN_RE = re.compile(r'<collision\b[^>]*\bname="([^"]*)"')
URI_RE = re.compile(r'<uri>[^<]*?/([^/<]+\.stl)</uri>', re.IGNORECASE)


def read_stl_aabb(path: Path) -> tuple[tuple[float, float, float],
                                       tuple[float, float, float]]:
    """Return (size_xyz, centroid_xyz) for a binary STL file."""
    with open(path, 'rb') as f:
        f.seek(80)
        n = struct.unpack('<I', f.read(4))[0]
        data = f.read(50 * n)

    mn = [float('inf')] * 3
    mx = [float('-inf')] * 3
    off = 0
    for _ in range(n):
        off += 12  # skip normal
        for _ in range(3):
            x, y, z = struct.unpack('<fff', data[off:off + 12])
            off += 12
            if x < mn[0]: mn[0] = x
            if y < mn[1]: mn[1] = y
            if z < mn[2]: mn[2] = z
            if x > mx[0]: mx[0] = x
            if y > mx[1]: mx[1] = y
            if z > mx[2]: mx[2] = z
        off += 2  # attribute count

    size = tuple(mx[i] - mn[i] for i in range(3))
    centroid = tuple((mx[i] + mn[i]) / 2 for i in range(3))
    return size, centroid


def rewrite_pose_line(line: str, dx: float, dy: float, dz: float) -> str:
    """Shift the translation part of a <pose>x y z r p y</pose> line."""
    m = POSE_RE.search(line)
    if not m:
        raise ValueError(f'No <pose> in line: {line!r}')
    parts = m.group(2).split()
    if len(parts) != 6:
        raise ValueError(f'Expected 6 pose components, got {parts}')
    x, y, z, r, p, yaw = (float(v) for v in parts)

    if any(abs(v) > ROTATION_TOLERANCE for v in (r, p, yaw)):
        raise ValueError(
            f'Non-identity rotation in pose ({r}, {p}, {yaw}) — '
            f'boxify would place the box incorrectly. Aborting.')

    new_parts = [f'{x + dx:.9g}', f'{y + dy:.9g}', f'{z + dz:.9g}',
                 '0', '0', '0']
    return line[:m.start(2)] + ' '.join(new_parts) + line[m.end(2):]


def box_geometry_block(indent: str, size: tuple[float, float, float]) -> list[str]:
    """Return the replacement <geometry> lines, matching surrounding indent."""
    inner = indent + '  '
    inner2 = inner + '  '
    sx, sy, sz = size
    return [
        f'{indent}<geometry>\n',
        f'{inner}<box>\n',
        f'{inner2}<size>{sx:.6f} {sy:.6f} {sz:.6f}</size>\n',
        f'{inner}</box>\n',
        f'{indent}</geometry>\n',
    ]


def leading_indent(line: str) -> str:
    return line[:len(line) - len(line.lstrip(' '))]


def boxify_block(block: list[str], pad_info: dict) -> list[str]:
    """Rewrite a single pad collision block. Returns new lines."""
    # Detect which pad from the URI or block text.
    block_text = ''.join(block)
    uri_match = URI_RE.search(block_text)
    if not uri_match:
        raise ValueError(f'No STL URI found in collision block: {block_text!r}')
    stl_name = uri_match.group(1)
    if stl_name not in pad_info:
        raise ValueError(f'Unknown pad mesh: {stl_name}')

    size = pad_info[stl_name]['size']
    cx, cy, cz = pad_info[stl_name]['centroid']

    out = []
    i = 0
    while i < len(block):
        line = block[i]

        if '<pose' in line and '</pose>' in line:
            out.append(rewrite_pose_line(line, cx, cy, cz))
            i += 1
            continue

        if '<geometry>' in line:
            geom_indent = leading_indent(line)
            # Skip through </geometry>
            j = i
            while j < len(block) and '</geometry>' not in block[j]:
                j += 1
            if j >= len(block):
                raise ValueError('Unterminated <geometry> in collision block')
            out.extend(box_geometry_block(geom_indent, size))
            i = j + 1
            continue

        out.append(line)
        i += 1

    return out


def boxify(lines: list[str], pad_info: dict) -> tuple[list[str], int]:
    out = []
    i = 0
    rewritten = 0
    while i < len(lines):
        line = lines[i]
        m = COLLISION_OPEN_RE.search(line)
        if m is None or 'adhesive_pad' not in m.group(1):
            out.append(line)
            i += 1
            continue

        block = [line]
        j = i + 1
        while j < len(lines) and '</collision>' not in lines[j]:
            block.append(lines[j])
            j += 1
        if j >= len(lines):
            raise ValueError(f'Unterminated <collision> at line {i + 1}')
        block.append(lines[j])

        out.extend(boxify_block(block, pad_info))
        rewritten += 1
        i = j + 1

    return out, rewritten


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Replace adhesive-pad mesh collisions with box primitives.')
    parser.add_argument('sdf', type=Path, help='Path to SDF file')
    parser.add_argument('--assets-dir', type=Path, default=None,
                        help='Directory containing the pad STL files '
                             '(default: <sdf_dir>/assets)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print stats; do not write')
    args = parser.parse_args()

    if not args.sdf.exists():
        print(f'ERROR: {args.sdf} not found.', file=sys.stderr)
        return 1

    assets = args.assets_dir or (args.sdf.parent / 'assets')
    pad_info = {}
    for stl_name in ('inner_adhesive_pad.stl', 'outer_adhesive_pad.stl'):
        stl = assets / stl_name
        if not stl.exists():
            print(f'ERROR: expected STL at {stl} (use --assets-dir to override)',
                  file=sys.stderr)
            return 1
        size, centroid = read_stl_aabb(stl)
        pad_info[stl_name] = {'size': size, 'centroid': centroid}
        print(f'{stl_name}:')
        print(f'  box size:    ({size[0]:.6f}, {size[1]:.6f}, {size[2]:.6f}) m')
        print(f'  pose shift:  ({centroid[0]:+.6f}, {centroid[1]:+.6f}, {centroid[2]:+.6f}) m')

    with open(args.sdf) as f:
        lines = f.readlines()

    try:
        new_lines, n = boxify(lines, pad_info)
    except ValueError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1

    print(f'\nInput:         {args.sdf}')
    print(f'Rewrote:       {n} pad collision blocks')

    if args.dry_run:
        print('(dry-run: no file written)')
        return 0

    backup = args.sdf.with_suffix(args.sdf.suffix + '.bak-stripped')
    if not backup.exists():
        shutil.copy2(args.sdf, backup)
        print(f'Backup:        {backup}')
    else:
        print(f'Backup:        {backup} (already exists — not overwritten)')

    with open(args.sdf, 'w') as f:
        f.writelines(new_lines)
    print(f'Wrote:         {args.sdf}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
