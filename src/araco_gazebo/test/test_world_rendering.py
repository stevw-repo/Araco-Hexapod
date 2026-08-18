# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import math
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


def test_flat_ground_loads_rendering_sensor_and_imu_systems():
    world = ET.parse(ROOT / 'worlds/flat_ground_v0.sdf').getroot().find('world')
    assert world is not None
    plugins = {plugin.attrib['name']: plugin for plugin in world.findall('plugin')}
    sensors = plugins['gz::sim::systems::Sensors']
    assert sensors.attrib['filename'] == 'gz-sim-sensors-system'
    assert sensors.findtext('render_engine') == 'ogre2'
    assert plugins['gz::sim::systems::Imu'].attrib['filename'] == 'gz-sim-imu-system'


def test_rgbd_validation_world_preserves_the_sensor_and_physics_contract():
    world = ET.parse(
        ROOT / 'worlds/rgbd_validation_v0.sdf').getroot().find('world')
    assert world is not None
    assert world.attrib['name'] == 'araco_rgbd_validation'
    assert world.findtext('physics/max_step_size') == '0.001'
    assert world.findtext('physics/real_time_factor') == '1.0'
    assert world.findtext('gravity') == '0 0 -9.80665'
    plugins = {plugin.attrib['name']: plugin for plugin in world.findall('plugin')}
    assert plugins['gz::sim::systems::Sensors'].findtext('render_engine') == 'ogre2'
    assert plugins['gz::sim::systems::Imu'].attrib['filename'] == 'gz-sim-imu-system'


def test_rgbd_validation_world_has_asymmetric_rgb_and_depth_landmarks():
    world = ET.parse(
        ROOT / 'worlds/rgbd_validation_v0.sdf').getroot().find('world')
    assert world is not None
    models = {model.attrib['name']: model for model in world.findall('model')}
    assert {
        'ground_plane', 'floor_markers', 'arena_walls', 'east_checkerboard',
        'red_gate', 'cyan_steps', 'magenta_totem', 'yellow_pillars',
        'wall_relief',
    } <= set(models)

    checker_visuals = models['east_checkerboard'].findall('.//visual')
    assert len(checker_visuals) == 21
    checker_colors = {
        visual.findtext('material/diffuse') for visual in checker_visuals}
    assert checker_colors == {'0.02 0.02 0.02 1', '1 1 1 1'}

    landmark_geometry = {
        name: {
            child.tag
            for visual in models[name].findall('.//visual')
            for child in visual.find('geometry')
        }
        for name in ('red_gate', 'cyan_steps', 'magenta_totem', 'yellow_pillars')
    }
    assert landmark_geometry['red_gate'] == {'box'}
    assert landmark_geometry['cyan_steps'] == {'box'}
    assert landmark_geometry['magenta_totem'] == {'cylinder', 'sphere'}
    assert landmark_geometry['yellow_pillars'] == {'cylinder'}

    # The initial footprint remains clear; landmarks begin outside a 1.4 m radius.
    for name in landmark_geometry:
        x, y, *_ = map(float, models[name].findtext('pose').split())
        assert math.hypot(x, y) >= 1.4

    # RGB floor features must never create tiny physical ledges under the feet.
    assert models['floor_markers'].find('.//collision') is None

    acceptance_markers = {
        visual.attrib['name']: tuple(map(float, visual.findtext('pose').split()[:2]))
        for visual in models['floor_markers'].findall('.//visual')
        if visual.attrib['name'].startswith('acceptance_')
    }
    assert acceptance_markers == {
        'acceptance_east': (1.2, 0.0),
        'acceptance_north': (0.0, 1.2),
        'acceptance_west': (-1.2, 0.0),
        'acceptance_south': (0.0, -1.2),
        'acceptance_origin': (0.0, 0.0),
    }

    heading_markers = {
        visual.attrib['name']
        for visual in models['floor_markers'].findall('.//visual')
        if visual.attrib['name'].startswith('start_heading_')
    }
    assert heading_markers == {
        'start_heading_shaft', 'start_heading_left', 'start_heading_right'}
