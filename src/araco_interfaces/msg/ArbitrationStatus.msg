# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

# Typed arbiter outcome used by supervision to distinguish source loss from a
# deliberate higher-priority preemption. This is status, never motion authority.
std_msgs/Header header
uint64 selection_epoch
uint32 previous_source_id
uint32 selected_source_id
uint64 selected_activation_epoch
uint16 reason_code
bool deliberate_higher_priority_preemption
bool all_sources_released
uint32[] quarantined_source_ids
