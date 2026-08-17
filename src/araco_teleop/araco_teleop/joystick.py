# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Pure PXN-2113 Pro mapping with report-freshness authority."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class JoystickIntent:
    """One freshness-gated simulator intent."""

    active: bool = False
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    body_z: float = 0.0
    body_roll: float = 0.0
    body_pitch: float = 0.0
    body_yaw: float = 0.0


def _shaped_axis(value: float, deadzone: float) -> float:
    """Apply a centered dead zone and preserve full-scale output."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign((magnitude - deadzone) / (1.0 - deadzone), value)


class JoystickState:
    """Validate raw Joy arrays and map the legacy physical control roles."""

    def __init__(self, mapping: dict[str, object]):
        self._mapping = mapping
        self._axis_count = int(mapping['axis_count'])
        self._button_count = int(mapping['button_count'])
        self._deadzone = float(mapping['deadzone'])
        self._timeout_s = float(mapping['heartbeat_timeout_s'])
        self._roll_left = int(mapping['roll_left_button'])
        self._roll_right = int(mapping['roll_right_button'])
        self._roll_scale = float(mapping['roll_button_scale_rad'])
        self._axes = mapping['axes']
        self._raw_axes = tuple(0.0 for _ in range(self._axis_count))
        self._buttons = tuple(0 for _ in range(self._button_count))
        self._fresh_until = 0.0
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError('joystick deadzone must be in [0, 1)')
        if not 0 <= self._roll_left < self._button_count:
            raise ValueError('joystick roll-left button is out of range')
        if not 0 <= self._roll_right < self._button_count:
            raise ValueError('joystick roll-right button is out of range')
        if self._roll_left == self._roll_right:
            raise ValueError('joystick roll buttons must be distinct')
        if not math.isfinite(self._roll_scale) or self._roll_scale <= 0.0:
            raise ValueError('joystick roll-button scale must be positive and finite')

    def accept(
        self,
        axes: list[float] | tuple[float, ...],
        buttons: list[int] | tuple[int, ...],
        now: float | None = None,
    ) -> None:
        """Accept one complete finite device report."""
        stamp = time.monotonic() if now is None else now
        if len(axes) != self._axis_count or len(buttons) != self._button_count:
            self.invalidate()
            raise ValueError('joystick report dimensions do not match the mapping')
        if any(not math.isfinite(float(value)) or abs(float(value)) > 1.001
               for value in axes):
            self.invalidate()
            raise ValueError('joystick report contains an invalid axis')
        if any(isinstance(value, bool) or int(value) not in (0, 1)
               for value in buttons):
            self.invalidate()
            raise ValueError('joystick report contains an invalid button')
        self._raw_axes = tuple(float(value) for value in axes)
        self._buttons = tuple(int(value) for value in buttons)
        self._fresh_until = stamp + self._timeout_s

    def invalidate(self) -> None:
        """Release authority immediately."""
        self._fresh_until = 0.0
        self._buttons = tuple(0 for _ in range(self._button_count))

    def _axis(self, role: str) -> float:
        specification = self._axes[role]
        value = self._raw_axes[int(specification['index'])]
        value = _shaped_axis(value, self._deadzone)
        if specification.get('invert', False):
            value = -value
        return value * float(specification['scale'])

    def sample(self, now: float | None = None) -> JoystickIntent:
        """Return a complete intent while the device report stream is fresh."""
        stamp = time.monotonic() if now is None else now
        if stamp > self._fresh_until:
            return JoystickIntent()
        throttle = max(-1.0, min(1.0, self._raw_axes[
            int(self._axes['body_height']['index'])]))
        positive_end_is_zero = bool(
            self._axes['body_height']['positive_end_is_zero'])
        height_axis = throttle if positive_end_is_zero else -throttle
        body_z = (height_axis - 1.0) * 0.5 * float(
            self._axes['body_height']['range_m'])
        roll_direction = (
            self._buttons[self._roll_right] - self._buttons[self._roll_left])
        return JoystickIntent(
            active=True,
            vx=self._axis('forward'),
            vy=self._axis('lateral'),
            wz=self._axis('walking_yaw'),
            body_z=body_z,
            body_roll=roll_direction * self._roll_scale,
            body_pitch=self._axis('body_pitch'),
            body_yaw=self._axis('posture_yaw'),
        )
