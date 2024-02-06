// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "main/Commands/UtilityFunctions/UtilityFunctions.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/Shooter/Shooter.h"
#include "main/Subsystems/Storage/Storage.h"
#include "main/Commands/StorageCommand/StorageCommand.h"


frc2::CommandPtr VisionSourceGrabCommand(SuperStructure* superStucture, Shooter* shooter, Storage* storage);