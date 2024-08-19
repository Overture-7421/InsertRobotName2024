// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Storage/Storage.h"

frc2::CommandPtr ClosedCommand(SuperStructure* superStructure, Intake* intake, Storage* storage, Shooter* shooter);
