# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

# Truth classification for each channel of the standard JointState stream.
uint8 SOURCE_UNAVAILABLE=0
uint8 SOURCE_SIMULATED_PHYSICS=1
uint8 SOURCE_HARDWARE_SENSOR=2
uint8 SOURCE_COMMAND_DERIVED=3
uint8 SOURCE_ESTIMATOR=4

std_msgs/Header header
uint64 provenance_epoch
string[] joint_names
uint8[] position_source
uint8[] velocity_source
uint8[] effort_source
