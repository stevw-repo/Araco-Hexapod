# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

from pathlib import Path
import xml.etree.ElementTree as ET


def test_package_metadata_matches_package_directory():
    package_root = Path(__file__).resolve().parents[1]
    repository_root = package_root.parent.parent
    manifest_root = ET.parse(package_root / 'package.xml').getroot()

    assert manifest_root.findtext('name') == package_root.name
    assert manifest_root.findtext('version') == '0.1.0'
    license_element = manifest_root.find('license')
    assert license_element is not None
    assert license_element.text == 'MIT'
    assert license_element.attrib == {'file': 'LICENSE'}
    assert (package_root / 'LICENSE').read_bytes() == (
        repository_root / 'LICENSE'
    ).read_bytes()


def test_project_source_has_exact_license_header():
    package_root = Path(__file__).resolve().parents[1]
    source_files = [package_root / 'CMakeLists.txt', *package_root.rglob('*.py')]
    for source_file in source_files:
        source = source_file.read_text(encoding='utf-8')
        assert 'Copyright 2026 Araco Hexapod contributors' in source
        assert 'SPDX-License-Identifier: MIT' in source
