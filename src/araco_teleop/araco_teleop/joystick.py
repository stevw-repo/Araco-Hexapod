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
    gimbal_yaw: float = 0.0

    def controls_centered(self) -> bool:
        """Whether a source-session recovery can occur without surprise motion."""
        # Height is retained trim; it does not create walking or posture motion.
        return all(abs(value) <= 1.0e-12 for value in (
            self.vx, self.vy, self.wz, self.body_roll,
            self.body_pitch, self.body_yaw, self.gimbal_yaw))


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
        self._publication_period_s = 1.0 / float(mapping['publication_rate_hz'])
        self._roll_left = int(mapping['roll_left_button'])
        self._roll_right = int(mapping['roll_right_button'])
        self._roll_scale = float(mapping['roll_button_scale_rad'])
        self._axes = mapping['axes']
        response = mapping['control_response']
        self._response_period_s = float(response['reference_period_s'])
        self._normal_error_fraction = float(response['normal_error_fraction'])
        self._height_error_fraction = float(response['height_error_fraction'])
        self._raw_axes = tuple(0.0 for _ in range(self._axis_count))
        self._buttons = tuple(0 for _ in range(self._button_count))
        self._fresh_until = 0.0
        self._last_sample_stamp: float | None = None
        self._filtered = {
            role: 0.0 for role in (
                'vx', 'vy', 'wz', 'body_z', 'body_roll',
                'body_pitch', 'axis4')
        }
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
        if not math.isfinite(self._response_period_s) or self._response_period_s <= 0.0:
            raise ValueError('control-response reference period must be positive and finite')
        for name, fraction in (
            ('normal', self._normal_error_fraction),
            ('height', self._height_error_fraction),
        ):
            if not math.isfinite(fraction) or not 0.0 < fraction < 1.0:
                raise ValueError(
                    f'{name} control-response error fraction must be in (0, 1)')
        posture_axis = self._axes['posture_yaw']
        gimbal_axis = self._axes['gimbal_yaw']
        if (
            int(posture_axis['index']) != int(gimbal_axis['index'])
            or bool(posture_axis.get('invert', False)) !=
            bool(gimbal_axis.get('invert', False))
        ):
            raise ValueError(
                'posture yaw and gimbal yaw must share one physical axis and polarity')

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
        self._reset_response()

    def _reset_response(self) -> None:
        """Reset all retained command response after authority is lost."""
        for role in self._filtered:
            self._filtered[role] = 0.0
        self._last_sample_stamp = None

    def reset_motion_response(self) -> None:
        """Remove residual motion before a centered source-session recovery."""
        for role in self._filtered:
            if role != 'body_z':
                self._filtered[role] = 0.0

    def _axis(self, role: str) -> float:
        return self._unit_axis(role) * float(self._axes[role]['scale'])

    def _unit_axis(self, role: str) -> float:
        specification = self._axes[role]
        value = self._raw_axes[int(specification['index'])]
        value = _shaped_axis(value, self._deadzone)
        if specification.get('invert', False):
            value = -value
        return value

    def _translation(self) -> tuple[float, float]:
        """Map the left stick as one radially bounded legacy vector."""
        forward = self._unit_axis('forward')
        lateral = self._unit_axis('lateral')
        magnitude = math.hypot(forward, lateral)
        if magnitude > 1.0:
            forward /= magnitude
            lateral /= magnitude
        return (
            forward * float(self._axes['forward']['scale']),
            lateral * float(self._axes['lateral']['scale']),
        )

    def _targets(self) -> dict[str, float]:
        """Return bounded unfiltered targets in command units."""
        throttle = max(-1.0, min(1.0, self._raw_axes[
            int(self._axes['body_height']['index'])]))
        positive_end_is_zero = bool(
            self._axes['body_height']['positive_end_is_zero'])
        height_axis = throttle if positive_end_is_zero else -throttle
        body_z = (height_axis - 1.0) * 0.5 * float(
            self._axes['body_height']['range_m'])
        roll_direction = (
            self._buttons[self._roll_right] - self._buttons[self._roll_left])
        vx, vy = self._translation()
        return {
            'vx': vx,
            'vy': vy,
            'wz': self._axis('walking_yaw'),
            'body_z': body_z,
            'body_roll': roll_direction * self._roll_scale,
            'body_pitch': self._axis('body_pitch'),
            # Filter this physical axis once. Body and gimbal outputs are
            # scaled from the same state so their normalized motion cannot
            # diverge.
            'axis4': self._unit_axis('posture_yaw'),
        }

    def raw_controls_centered(self) -> bool:
        """Whether raw motion controls are centered, excluding retained height."""
        targets = self._targets()
        return all(abs(targets[role]) <= 1.0e-12 for role in (
            'vx', 'vy', 'wz', 'body_roll', 'body_pitch', 'axis4'))

    def _advance_response(
        self, targets: dict[str, float], elapsed_s: float,
    ) -> None:
        """Apply the legacy error fraction independent of the caller rate."""
        elapsed_s = max(0.0, elapsed_s)
        for role, target in targets.items():
            fraction = (
                self._height_error_fraction if role == 'body_z'
                else self._normal_error_fraction)
            alpha = -math.expm1(
                math.log1p(-fraction) * elapsed_s / self._response_period_s)
            self._filtered[role] += alpha * (target - self._filtered[role])

    def sample(self, now: float | None = None) -> JoystickIntent:
        """Return a complete intent while the device report stream is fresh."""
        stamp = time.monotonic() if now is None else now
        if stamp > self._fresh_until:
            self._reset_response()
            return JoystickIntent()
        elapsed_s = (
            self._publication_period_s if self._last_sample_stamp is None
            else stamp - self._last_sample_stamp)
        self._last_sample_stamp = stamp
        self._advance_response(self._targets(), elapsed_s)
        axis4 = self._filtered['axis4']
        return JoystickIntent(
            active=True,
            vx=self._filtered['vx'],
            vy=self._filtered['vy'],
            wz=self._filtered['wz'],
            body_z=self._filtered['body_z'],
            body_roll=self._filtered['body_roll'],
            body_pitch=self._filtered['body_pitch'],
            body_yaw=axis4 * float(self._axes['posture_yaw']['scale']),
            gimbal_yaw=axis4 * float(self._axes['gimbal_yaw']['scale']),
        )
