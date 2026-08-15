# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

from pathlib import Path
import xml.etree.ElementTree as ET


EXPECTED_PROJECT_DEPENDENCIES = {
    'araco_interfaces': set(),
    'araco_description': set(),
    'araco_kinematics': set(),
    'araco_locomotion': {
        'araco_description',
        'araco_interfaces',
        'araco_kinematics',
    },
    'araco_supervision': {'araco_interfaces'},
    'araco_teleop': {'araco_interfaces'},
    'araco_gazebo': {'araco_description'},
    'araco_bringup': {
        'araco_description',
        'araco_gazebo',
        'araco_locomotion',
        'araco_supervision',
        'araco_teleop',
    },
    'araco_system_tests': {'araco_bringup'},
}

DEPENDENCY_TAGS = (
    'depend',
    'build_depend',
    'build_export_depend',
    'exec_depend',
    'test_depend',
)


def _project_dependencies(package_root):
    manifest = ET.parse(package_root / 'package.xml').getroot()
    all_project_names = set(EXPECTED_PROJECT_DEPENDENCIES)
    dependencies = {
        element.text
        for tag in DEPENDENCY_TAGS
        for element in manifest.findall(tag)
    }
    return dependencies & all_project_names


def test_project_dependency_edges_match_architecture():
    source_root = Path(__file__).resolve().parents[2]
    actual = {
        package_name: _project_dependencies(source_root / package_name)
        for package_name in EXPECTED_PROJECT_DEPENDENCIES
    }
    assert actual == EXPECTED_PROJECT_DEPENDENCIES


def test_project_dependency_graph_is_acyclic():
    source_root = Path(__file__).resolve().parents[2]
    graph = {
        package_name: _project_dependencies(source_root / package_name)
        for package_name in EXPECTED_PROJECT_DEPENDENCIES
    }
    complete = set()
    active = set()

    def visit(package_name):
        assert package_name not in active, f'dependency cycle at {package_name}'
        if package_name in complete:
            return
        active.add(package_name)
        for dependency in graph[package_name]:
            visit(dependency)
        active.remove(package_name)
        complete.add(package_name)

    for package_name in graph:
        visit(package_name)
