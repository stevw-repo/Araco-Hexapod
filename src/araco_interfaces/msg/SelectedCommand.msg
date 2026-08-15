# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

# Arbiter-selected command. Selection is not authorization to move.
std_msgs/Header header
uint64 selection_epoch
bool has_selection
uint32 source_id
builtin_interfaces/Time source_stamp
uint64 source_sequence
araco_interfaces/MotionIntent intent
