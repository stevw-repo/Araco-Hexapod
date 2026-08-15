# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

# Sole high-level command accepted by locomotion.
uint8 DISPOSITION_HOLD=0
uint8 DISPOSITION_EXECUTE=1
uint8 DISPOSITION_LIMITED=2
uint8 DISPOSITION_CONTROLLED_STOP=3

std_msgs/Header header
uint64 safety_epoch
uint64 selection_epoch
uint8 disposition
uint16 reason_code
uint32 source_id
builtin_interfaces/Time source_stamp
uint64 source_sequence
araco_interfaces/MotionIntent intent
