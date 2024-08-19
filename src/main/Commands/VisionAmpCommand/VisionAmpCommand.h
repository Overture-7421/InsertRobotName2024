// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Shooter/Shooter.h"

frc2::CommandPtr VisionAmpCommand(SuperStructure* superStucture, Shooter* shooter);

// frc2::CommandPtr VisionAmpCommand(Chassis* chassis);
