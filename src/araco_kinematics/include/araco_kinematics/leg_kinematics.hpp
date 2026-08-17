// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_KINEMATICS__LEG_KINEMATICS_HPP_
#define ARACO_KINEMATICS__LEG_KINEMATICS_HPP_

#include <array>
#include <cstdint>
#include <utility>

namespace araco_kinematics
{

struct Point3
{
  double x;
  double y;
  double z;
};

struct LegGeometry
{
  double coxa_m;
  double femur_m;
  double tibia_m;
  double foot_m;
};

struct JointLimit
{
  double lower_rad;
  double upper_rad;
};

using JointVector = std::array<double, 4>;
using JointLimits = std::array<JointLimit, 4>;

enum class Branch : std::uint8_t
{
  kKneeDown = 0,
  kKneeUp = 1,
};

enum class Status : std::uint8_t
{
  kValid = 0,
  kNearLimit = 1,
  kUnreachable = 2,
  kSingular = 3,
  kInvalidInput = 4,
  kJointLimit = 5,
};

struct SolverOptions
{
  double position_tolerance_m;
  double singularity_threshold;
  double near_limit_margin_rad;
};

struct ForwardResult
{
  Status status{Status::kInvalidInput};
  Point3 foot_position_m{};
  double foot_pitch_rad{0.0};
};

struct InverseResult
{
  Status status{Status::kInvalidInput};
  JointVector joints_rad{};
  double position_error_m{0.0};
  double pitch_error_rad{0.0};
};

class LegKinematics
{
public:
  LegKinematics(LegGeometry geometry, JointLimits limits, SolverOptions options);

  [[nodiscard]] bool valid_configuration() const;
  [[nodiscard]] ForwardResult forward(const JointVector & joints_rad) const;
  [[nodiscard]] InverseResult inverse(
    const Point3 & foot_position_m,
    double foot_pitch_rad,
    Branch branch) const;

private:
  LegGeometry geometry_;
  JointLimits limits_;
  SolverOptions options_;
  bool valid_configuration_{false};
};

}  // namespace araco_kinematics

#endif  // ARACO_KINEMATICS__LEG_KINEMATICS_HPP_
