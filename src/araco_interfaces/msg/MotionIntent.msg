# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

# Desired motion targets. All linear values are SI metres/metres per second;
# angular values are radians/radians per second in REP-103 body axes.
uint8 GAIT_STAND=0
uint8 GAIT_TRIPOD=1

uint8 gait
geometry_msgs/Twist planar_velocity
geometry_msgs/Pose body_pose_offset
# Independently controlled camera-gimbal yaw target. It remains inside the
# supervised command transaction and is never smuggled through a reserved
# Twist or Pose component.
float64 gimbal_yaw_rad
