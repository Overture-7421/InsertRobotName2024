// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommandSmooth.h"
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/InstantCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ClosedCommandSmooth::ClosedCommandSmooth(SuperStructure* superStructure, Intake* intake, Storage* storage, Shooter* shooter) {
	AddCommands(
		frc2::InstantCommand([=] {
		storage->setVoltage(0_V);
		intake->setVoltage(0_V);
		shooter->setVelocityVoltage(ShooterConstants::IdleSpeed);
	}), frc2::SequentialCommandGroup(
		frc2::InstantCommand([=] {superStructure->setTargetCoord({ superStructure->getCurrentState().lowerAngle, -7.0 });}),
		frc2::WaitCommand(0.3_s),
		frc2::InstantCommand([=] {superStructure->setTargetCoord({ -26,-7.0 });})
	));
}
