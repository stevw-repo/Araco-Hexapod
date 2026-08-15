# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

# Untrusted source candidate. Source identity, priority, and freshness timeout
# come from supervisor configuration, never from this message.
std_msgs/Header header
uint64 sequence
bool active
araco_interfaces/MotionIntent intent
