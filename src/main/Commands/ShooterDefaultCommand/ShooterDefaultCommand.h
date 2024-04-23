// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <pathplanner/lib/util/GeometryUtil.h>

#include "Subsystems/Shooter/Shooter.h"
#include "Commands/VisionSpeakerCommand/Constants.h"
#include "Commands/UtilityFunctions/UtilityFunctions.h"

class ShooterDefaultCommand
    : public frc2::CommandHelper<frc2::Command, ShooterDefaultCommand> {
 public:
  ShooterDefaultCommand(Chassis* chassis, Shooter* shooter, const frc::AprilTagFieldLayout* layout);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  Shooter* shooter;
  frc::Translation2d targetLocation;
  Chassis* chassis;
  const frc::AprilTagFieldLayout* layout;
};
