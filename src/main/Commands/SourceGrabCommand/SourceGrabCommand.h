// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "main/Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "main/Commands/ShooterCommand/ShooterCommand.h"
#include "main/Commands/StorageCommand/StorageCommand.h"

frc2::CommandPtr SourceGrabCommand(SuperStructure* superStructure, Shooter* shooter, Storage* storage);
