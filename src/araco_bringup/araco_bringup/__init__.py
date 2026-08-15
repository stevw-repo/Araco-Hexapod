# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Deterministic configuration composition for the Araco runtime."""

from .composer import compose_profile
from .composer import CompositionError

__all__ = ['CompositionError', 'compose_profile']
