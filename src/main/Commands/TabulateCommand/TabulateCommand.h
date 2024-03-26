// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/Shooter/Shooter.h"
#include "main/Subsystems/Chassis/Chassis.h"

#include "main/Commands/UtilityFunctions/UtilityFunctions.h"
#include "main/Commands/VisionSpeakerCommand/Constants.h"

class TabulateCommand
    : public frc2::CommandHelper<frc2::Command, TabulateCommand> {
 public:
  TabulateCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, const frc::AprilTagFieldLayout* layout);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  Chassis* chassis;
  SuperStructure* superStructure;
  Shooter* shooter;
  const frc::AprilTagFieldLayout* layout;

  frc::Translation2d targetLocation;
};
