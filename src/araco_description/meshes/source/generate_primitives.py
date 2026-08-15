#!/usr/bin/env python3
# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Generate reviewed ASCII STL visual proxies from the canonical model registry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


TRIANGLES = (
    (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
    (0, 1, 5), (0, 5, 4), (3, 7, 6), (3, 6, 2),
    (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5),
)


def _vertices(center, size):
    half = [value / 2.0 for value in size]
    return [
        (center[0] + sx * half[0], center[1] + sy * half[1], center[2] + sz * half[2])
        for sx, sy, sz in (
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
        )
    ]


def _append_box(lines, center, size):
    vertices = _vertices(center, size)
    for first, second, third in TRIANGLES:
        lines.extend([
            '  facet normal 0 0 0',
            '    outer loop',
            '      vertex ' + ' '.join(format(value, '.15g') for value in vertices[first]),
            '      vertex ' + ' '.join(format(value, '.15g') for value in vertices[second]),
            '      vertex ' + ' '.join(format(value, '.15g') for value in vertices[third]),
            '    endloop',
            '  endfacet',
        ])


def _write_stl(path, name, boxes):
    lines = [f'solid {name}']
    for box in boxes:
        _append_box(lines, box['center_xyz_m'], box['size_m'])
    lines.append(f'endsolid {name}')
    path.write_text('\n'.join(lines) + '\n', encoding='ascii')


def main():
    """Generate one deterministic mesh for each canonical geometry class."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('canonical_model')
    parser.add_argument('output_directory')
    arguments = parser.parse_args()
    document = json.loads(Path(arguments.canonical_model).read_text(encoding='utf-8'))
    output = Path(arguments.output_directory)
    output.mkdir(parents=True, exist_ok=True)
    for name, geometry in document['data']['geometry_classes'].items():
        boxes = geometry.get('visual_boxes', [{
            'center_xyz_m': geometry['collision_origin_xyz_m'],
            'size_m': geometry['collision_size_m'],
        }])
        _write_stl(
            output / f'{name}.stl',
            name,
            boxes,
        )


if __name__ == '__main__':
    main()
