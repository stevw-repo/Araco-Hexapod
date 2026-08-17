# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

# Deterministic locomotion state; never an actuator command.
uint8 MODE_INACTIVE=0
uint8 MODE_HOLDING=1
uint8 MODE_STANDING=2
uint8 MODE_STARTING=3
uint8 MODE_WALKING=4
uint8 MODE_STOPPING=5
uint8 MODE_FAULT=6

uint8 LEG_VALID=0
uint8 LEG_NEAR_LIMIT=1
uint8 LEG_UNREACHABLE=2
uint8 LEG_INVALID=3

std_msgs/Header header
uint64 status_sequence
uint64 processed_safety_epoch
uint64 processed_selection_epoch
uint8 mode
uint8 gait
float64 gait_phase
uint64 gait_cycle
float64 gait_cadence_hz
float64 gait_maximum_stride_scale
float64 gait_maximum_clearance_m
float64 gait_applied_velocity_scale
uint8[6] leg_kinematic_status
bool trajectory_valid
uint16 reason_code
