// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/SuperStructure/SuperStructure.h"


class SuperStructureMoveByDistance
    : public frc2::CommandHelper<frc2::Command, SuperStructureMoveByDistance> {
 public:
   struct Profile {
    SuperStructureState startingState;
    SuperStructureState targetState;
    units::meter_t profileActivationDistance;
  };

  SuperStructureMoveByDistance(SuperStructure* superStructure, Profile profile, std::function<units::meter_t()> distanceToTargetProvider);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  SuperStructure* superStructure;

  std::function<units::meter_t()> distanceToTargetProvider;
  units::meter_t distanceToTarget;

  units::degree_t upperAngleTravel = 0_deg;
  units::degree_t lowerAngleTravel = 0_deg;
  
  Profile profile;
};
