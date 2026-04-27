#!/usr/bin/env python3
"""
Strip non-essential <collision> blocks from a robot SDF.

The snake SDF ships with ~112 mesh collisions — one per mechanical part —
but only the adhesive pads ever touch the ground. DART spends most of its
CPU in mesh-vs-plane contact checks on parts that couldn't possibly collide.
Removing them cuts contact-resolution cost massively without changing
observable locomotion (internal parts are rigidly constrained by joints
regardless of collision geometry).

Reads an SDF, keeps only <collision> blocks whose `name` attribute matches
one of the --keep-pattern substrings, and writes the result back. The
original is backed up to <input>.bak on first run (never overwritten).

Usage:
    python3 scripts/strip_collisions.py sdf/snake/snake.sdf
    python3 scripts/strip_collisions.py sdf/snake/snake.sdf --keep-pattern adhesive_pad --dry-run
"""
import argparse
import re
import shutil
import sys
from pathlib import Path


COLLISION_OPEN_RE = re.compile(r'<collision\b[^>]*\bname="([^"]*)"')


def strip_collisions(lines: list[str], keep_patterns: list[str]) -> tuple[list[str], int, int]:
    """Return (kept_lines, kept_count, removed_count)."""
    out = []
    i = 0
    kept = 0
    removed = 0

    while i < len(lines):
        line = lines[i]
        m = COLLISION_OPEN_RE.search(line)
        if m is None:
            out.append(line)
            i += 1
            continue

        # Found a collision block. Collect until </collision>.
        name = m.group(1)
        block = [line]
        j = i + 1
        while j < len(lines) and '</collision>' not in lines[j]:
            block.append(lines[j])
            j += 1
        if j >= len(lines):
            raise ValueError(f'Unterminated <collision> starting at line {i + 1}')
        block.append(lines[j])  # closing tag line
        i = j + 1

        # Keep the block iff its name contains any of the keep patterns.
        if any(pat in name for pat in keep_patterns):
            out.extend(block)
            kept += 1
        else:
            removed += 1

    return out, kept, removed


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Strip non-essential <collision> blocks from an SDF.')
    parser.add_argument('sdf', type=Path, help='Path to SDF file')
    parser.add_argument('--keep-pattern', action='append', default=None,
                        help='Substring to match against collision name. '
                             'Repeatable. Default: adhesive_pad')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print stats; do not write')
    args = parser.parse_args()

    keep_patterns = args.keep_pattern or ['adhesive_pad']

    if not args.sdf.exists():
        print(f'ERROR: {args.sdf} not found.', file=sys.stderr)
        return 1

    with open(args.sdf) as f:
        lines = f.readlines()

    try:
        new_lines, kept, removed = strip_collisions(lines, keep_patterns)
    except ValueError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1

    print(f'Input:     {args.sdf}')
    print(f'Patterns:  {keep_patterns}')
    print(f'Kept:      {kept} collision blocks')
    print(f'Removed:   {removed} collision blocks')

    if args.dry_run:
        print('(dry-run: no file written)')
        return 0

    backup = args.sdf.with_suffix(args.sdf.suffix + '.bak')
    if not backup.exists():
        shutil.copy2(args.sdf, backup)
        print(f'Backup:    {backup}')
    else:
        print(f'Backup:    {backup} (already exists — not overwritten)')

    with open(args.sdf, 'w') as f:
        f.writelines(new_lines)
    print(f'Wrote:     {args.sdf}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
