# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import json
from pathlib import Path

from araco_bringup.strict_yaml import canonical_json_bytes
from araco_bringup.strict_yaml import load_strict
from araco_bringup.strict_yaml import StrictYamlError
import pytest


FIXTURES = Path(__file__).parent / 'fixtures'


@pytest.mark.parametrize(
    'name',
    [
        'duplicate_key.yaml',
        'alias.yaml',
        'nonfinite.yaml',
        'timestamp.yaml',
        'multiple_documents.yaml',
    ],
)
def test_forbidden_yaml_constructs_fail_closed(name):
    with pytest.raises(StrictYamlError):
        load_strict(FIXTURES / name)


def test_normalization_is_order_independent(tmp_path):
    first = tmp_path / 'first.yaml'
    second = tmp_path / 'second.yaml'
    first.write_text('{"b":2,"a":[true,1.0]}\n', encoding='utf-8')
    second.write_text('a: [true, 1.0]\nb: 2\n', encoding='utf-8')

    assert canonical_json_bytes(load_strict(first)) == canonical_json_bytes(
        load_strict(second)
    )
    assert json.loads(canonical_json_bytes(load_strict(first))) == {
        'a': [True, 1.0],
        'b': 2,
    }
