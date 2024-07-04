// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/Intake/Intake.h"

frc2::CommandPtr GroundGrabCommand(SuperStructure* superStructure, Storage* storage, Intake* intake, bool ignoreSensor = false);
