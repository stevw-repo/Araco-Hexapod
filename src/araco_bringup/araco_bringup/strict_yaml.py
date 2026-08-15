# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Strict JSON-compatible YAML loading and canonical serialization."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import yaml
from yaml.nodes import MappingNode
from yaml.nodes import ScalarNode
from yaml.nodes import SequenceNode
from yaml.tokens import AliasToken
from yaml.tokens import AnchorToken
from yaml.tokens import DocumentEndToken
from yaml.tokens import DocumentStartToken
from yaml.tokens import TagToken


class StrictYamlError(ValueError):
    """Raised when source configuration is outside the accepted YAML subset."""


_NUMBER = re.compile(r'-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?\Z')
_INTEGER = re.compile(r'-?(?:0|[1-9][0-9]*)\Z')
_TIMESTAMP = re.compile(r'[0-9]{4}-[0-9]{2}-[0-9]{2}(?:[Tt ].*)?\Z')
_AMBIGUOUS = {
    'y', 'yes', 'n', 'no', 'on', 'off', 'null', '~',
    '.nan', '.inf', '+.inf', '-.inf',
}


def _convert_scalar(node: ScalarNode, *, mapping_key: bool = False) -> Any:
    value = node.value
    if mapping_key:
        if node.style is None and (_NUMBER.fullmatch(value) or value.lower() in _AMBIGUOUS):
            raise StrictYamlError(f'non-string or ambiguous mapping key {value!r}')
        return value
    if node.style is not None:
        return value
    lowered = value.lower()
    if value == 'true':
        return True
    if value == 'false':
        return False
    if value == 'null':
        return None
    if lowered in _AMBIGUOUS or _TIMESTAMP.fullmatch(value):
        raise StrictYamlError(f'ambiguous YAML scalar {value!r}')
    if _INTEGER.fullmatch(value):
        return int(value)
    if _NUMBER.fullmatch(value):
        number = float(value)
        if not math.isfinite(number):
            raise StrictYamlError(f'non-finite scalar {value!r}')
        return number
    if value == '':
        raise StrictYamlError('empty implicit scalar; use an explicit JSON value')
    return value


def _convert_node(node: Any) -> Any:
    if isinstance(node, ScalarNode):
        return _convert_scalar(node)
    if isinstance(node, SequenceNode):
        return [_convert_node(item) for item in node.value]
    if isinstance(node, MappingNode):
        result = {}
        for key_node, value_node in node.value:
            if not isinstance(key_node, ScalarNode):
                raise StrictYamlError('mapping keys must be strings')
            key = _convert_scalar(key_node, mapping_key=True)
            if key == '<<':
                raise StrictYamlError('YAML merge keys are forbidden')
            if key in result:
                raise StrictYamlError(f'duplicate mapping key {key!r}')
            result[key] = _convert_node(value_node)
        return result
    raise StrictYamlError(f'unsupported YAML node {type(node).__name__}')


def load_strict(path: str | Path) -> Any:
    """Load one UTF-8 YAML document using the accepted JSON-compatible subset."""
    source_path = Path(path)
    text = source_path.read_text(encoding='utf-8')
    try:
        tokens = list(yaml.scan(text))
    except yaml.YAMLError as error:
        raise StrictYamlError(f'{source_path}: invalid YAML: {error}') from error
    forbidden = (AnchorToken, AliasToken, TagToken)
    for token in tokens:
        if isinstance(token, forbidden):
            raise StrictYamlError(
                f'{source_path}: anchors, aliases, and tags are forbidden'
            )
    document_markers = sum(
        isinstance(token, (DocumentStartToken, DocumentEndToken))
        for token in tokens
    )
    if document_markers:
        raise StrictYamlError(f'{source_path}: explicit or multiple documents are forbidden')
    try:
        nodes = list(yaml.compose_all(text, Loader=yaml.BaseLoader))
    except yaml.YAMLError as error:
        raise StrictYamlError(f'{source_path}: invalid YAML: {error}') from error
    if len(nodes) != 1 or nodes[0] is None:
        raise StrictYamlError(f'{source_path}: exactly one non-empty document is required')
    value = _convert_node(nodes[0])
    canonical_json_bytes(value)
    return value


def canonical_json_bytes(value: Any) -> bytes:
    """Serialize JSON-compatible data into the project's canonical byte form."""
    try:
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as error:
        raise StrictYamlError(f'value is not finite JSON-compatible data: {error}') from error
    return encoded.encode('utf-8')


def content_sha256(value: Any) -> str:
    """Hash normalized structured data."""
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()
