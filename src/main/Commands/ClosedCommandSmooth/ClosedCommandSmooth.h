// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "Commands/IntakeCommand/IntakeCommand.h"
#include "Commands/StorageCommand/StorageCommand.h"
#include "Commands/ShooterCommand/ShooterCommand.h"

class ClosedCommandSmooth
	: public frc2::CommandHelper<frc2::SequentialCommandGroup,
	ClosedCommandSmooth> {
public:
	ClosedCommandSmooth(SuperStructure* superStructure, Intake* intake, Storage* storage, Shooter* shooter);
};
