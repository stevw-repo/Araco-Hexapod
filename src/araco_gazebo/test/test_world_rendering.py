# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_flat_ground_has_explicit_directional_lighting_and_shadows():
    world = ET.parse(ROOT / 'worlds/flat_ground_v0.sdf').getroot().find('world')
    assert world is not None
    scene = world.find('scene')
    assert scene is not None
    assert scene.findtext('shadows') == 'true'
    assert scene.find('ambient') is not None
    sun = world.find("light[@name='sun']")
    assert sun is not None
    assert sun.attrib['type'] == 'directional'
    assert sun.findtext('cast_shadows') == 'true'
    assert sun.find('direction') is not None
    assert sun.find('diffuse') is not None
