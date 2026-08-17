# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

from araco_interfaces.action import SafetyTransition
from araco_interfaces.msg import ArbitrationStatus
from araco_interfaces.msg import CommandCandidate
from araco_interfaces.msg import JointStateProvenance
from araco_interfaces.msg import LocomotionStatus
from araco_interfaces.msg import MotionIntent
from araco_interfaces.msg import SafeCommand
from araco_interfaces.msg import SafetyStatus
from araco_interfaces.msg import SelectedCommand


def test_command_interface_fields():
    assert MotionIntent.get_fields_and_field_types() == {
        'gait': 'uint8',
        'planar_velocity': 'geometry_msgs/Twist',
        'body_pose_offset': 'geometry_msgs/Pose',
    }
    assert CommandCandidate.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'sequence': 'uint64',
        'active': 'boolean',
        'intent': 'araco_interfaces/MotionIntent',
    }
    assert SelectedCommand.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'selection_epoch': 'uint64',
        'has_selection': 'boolean',
        'source_id': 'uint32',
        'source_stamp': 'builtin_interfaces/Time',
        'source_sequence': 'uint64',
        'intent': 'araco_interfaces/MotionIntent',
    }
    assert SafeCommand.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'safety_epoch': 'uint64',
        'selection_epoch': 'uint64',
        'disposition': 'uint8',
        'reason_code': 'uint16',
        'source_id': 'uint32',
        'source_stamp': 'builtin_interfaces/Time',
        'source_sequence': 'uint64',
        'intent': 'araco_interfaces/MotionIntent',
    }


def test_status_interface_fields():
    assert ArbitrationStatus.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'selection_epoch': 'uint64',
        'previous_source_id': 'uint32',
        'selected_source_id': 'uint32',
        'selected_activation_epoch': 'uint64',
        'reason_code': 'uint16',
        'deliberate_higher_priority_preemption': 'boolean',
        'all_sources_released': 'boolean',
        'quarantined_source_ids': 'sequence<uint32>',
    }
    assert JointStateProvenance.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'provenance_epoch': 'uint64',
        'joint_names': 'sequence<string>',
        'position_source': 'sequence<uint8>',
        'velocity_source': 'sequence<uint8>',
        'effort_source': 'sequence<uint8>',
    }
    assert LocomotionStatus.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'status_sequence': 'uint64',
        'processed_safety_epoch': 'uint64',
        'processed_selection_epoch': 'uint64',
        'mode': 'uint8',
        'gait': 'uint8',
        'gait_phase': 'double',
        'gait_cycle': 'uint64',
        'gait_cadence_hz': 'double',
        'gait_maximum_stride_scale': 'double',
        'gait_maximum_clearance_m': 'double',
        'gait_applied_velocity_scale': 'double',
        'leg_kinematic_status': 'uint8[6]',
        'trajectory_valid': 'boolean',
        'reason_code': 'uint16',
    }
    assert SafetyStatus.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'safety_epoch': 'uint64',
        'state': 'uint8',
        'disposition': 'uint8',
        'reason_code': 'uint16',
        'selected_source_id': 'uint32',
        'readiness_mask': 'uint64',
        'required_readiness_mask': 'uint64',
        'fault_mask': 'uint64',
        'reset_required': 'boolean',
    }


def test_action_fields():
    assert SafetyTransition.Goal.get_fields_and_field_types() == {
        'request': 'uint8',
    }
    assert SafetyTransition.Result.get_fields_and_field_types() == {
        'accepted': 'boolean',
        'final_state': 'uint8',
        'reason_code': 'uint16',
    }
    assert SafetyTransition.Feedback.get_fields_and_field_types() == {
        'state': 'uint8',
        'reason_code': 'uint16',
    }


def test_command_and_provenance_constants():
    assert (MotionIntent.GAIT_STAND, MotionIntent.GAIT_TRIPOD) == (0, 1)
    assert (
        SafeCommand.DISPOSITION_HOLD,
        SafeCommand.DISPOSITION_EXECUTE,
        SafeCommand.DISPOSITION_LIMITED,
        SafeCommand.DISPOSITION_CONTROLLED_STOP,
    ) == (0, 1, 2, 3)
    assert (
        JointStateProvenance.SOURCE_UNAVAILABLE,
        JointStateProvenance.SOURCE_SIMULATED_PHYSICS,
        JointStateProvenance.SOURCE_HARDWARE_SENSOR,
        JointStateProvenance.SOURCE_COMMAND_DERIVED,
        JointStateProvenance.SOURCE_ESTIMATOR,
    ) == (0, 1, 2, 3, 4)


def test_locomotion_constants():
    assert (
        LocomotionStatus.MODE_INACTIVE,
        LocomotionStatus.MODE_HOLDING,
        LocomotionStatus.MODE_STANDING,
        LocomotionStatus.MODE_STARTING,
        LocomotionStatus.MODE_WALKING,
        LocomotionStatus.MODE_STOPPING,
        LocomotionStatus.MODE_FAULT,
    ) == tuple(range(7))
    assert (
        LocomotionStatus.LEG_VALID,
        LocomotionStatus.LEG_NEAR_LIMIT,
        LocomotionStatus.LEG_UNREACHABLE,
        LocomotionStatus.LEG_INVALID,
    ) == tuple(range(4))


def test_safety_constants():
    assert (
        SafetyStatus.STATE_INITIALIZING,
        SafetyStatus.STATE_INACTIVE,
        SafetyStatus.STATE_HOLDING,
        SafetyStatus.STATE_ENABLING,
        SafetyStatus.STATE_MOTION_ENABLED,
        SafetyStatus.STATE_STOPPING,
        SafetyStatus.STATE_FAULT_HOLD,
        SafetyStatus.STATE_SHUTTING_DOWN,
    ) == tuple(range(8))

    reason_names = [
        'REASON_NONE',
        'REASON_STARTUP',
        'REASON_INACTIVE',
        'REASON_HOLDING',
        'REASON_WAITING_FOR_SOURCE',
        'REASON_MANUAL_HOLD',
        'REASON_NO_SOURCE',
        'REASON_SOURCE_RELEASED',
        'REASON_SOURCE_STALE',
        'REASON_SOURCE_HANDOVER',
        'REASON_SOURCE_INVALID',
        'REASON_COMMAND_LIMITED',
        'REASON_SELECTED_COMMAND_STALE',
        'REASON_SAFE_COMMAND_STALE',
        'REASON_LOCOMOTION_NOT_READY',
        'REASON_LOCOMOTION_STALE',
        'REASON_KINEMATICS_INVALID',
        'REASON_JOINT_LIMIT',
        'REASON_JOINT_STATE_STALE',
        'REASON_JOINT_STATE_INVALID',
        'REASON_CONTROLLER_NOT_READY',
        'REASON_CONTROLLER_FAULT',
        'REASON_BACKEND_FAULT',
        'REASON_TIME_DISCONTINUITY',
        'REASON_SHUTDOWN_REQUESTED',
        'REASON_SOFTWARE_LATCHED_HOLD',
        'REASON_RESET_REQUIRED',
        'REASON_INTERNAL_ERROR',
        'REASON_POWER_WARNING',
        'REASON_POWER_CRITICAL',
        'REASON_SERIAL_FAULT',
    ]
    assert [getattr(SafetyStatus, name) for name in reason_names] == list(
        range(31)
    )

    assert (
        SafetyTransition.Goal.REQUEST_HOLD,
        SafetyTransition.Goal.REQUEST_ENABLE_MOTION,
        SafetyTransition.Goal.REQUEST_RESET_FAULT,
        SafetyTransition.Goal.REQUEST_SHUTDOWN,
        SafetyTransition.Goal.REQUEST_LATCHED_HOLD,
    ) == (1, 2, 3, 4, 5)
