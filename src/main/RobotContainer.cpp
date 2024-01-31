// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <iostream>

RobotContainer::RobotContainer() {
	auto alliance = frc::DriverStation::GetAlliance();

	// TODO: Delete if not needed in 2025, it just waits until there is an alliance assigned so pose flipping is done correctly.
	while (!alliance.has_value()) {
		alliance = frc::DriverStation::GetAlliance();
		std::cout << "Warning: Waiting for alliance color before starting robot..." << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	autoChooser.SetDefaultOption("None, null, nada", "None");
	autoChooser.AddOption("MiddleNote", "MiddleNote");

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	superStructure.SetDefaultCommand(frc2::cmd::RunOnce([this]() {superStructure.setTargetCoord({ -15, 0 });}, { &superStructure }));

	chassis.SetDefaultCommand(Drive(&chassis, &driver));
	superStructure.SetDefaultCommand(IdleSuperStructure(&intake, &superStructure));

	// Configure the button bindings
	resetAngleButton.WhileTrue(ResetAngle(&chassis).ToPtr());

	climbButton.WhileTrue(Climb(&chassis, &superStructure, &supportArms, &driver));
	shootTrap.WhileTrue(TrapShoot(&chassis, &superStructure, &supportArms, &shooter, &storage));

	intakePosition.OnTrue(StartIntake(&intake, &superStructure, &storage)).OnFalse(StopIntake(&intake, &superStructure, &storage));
	shootingPose.OnTrue(ShootingPose(&intake, &superStructure)).OnFalse(StopIntake(&intake, &superStructure, &storage));
	moveStorage.WhileTrue(MoveStorage(&storage, 1_V).ToPtr());
	moveStorageInverted.WhileTrue(MoveStorage(&storage, -1_V).ToPtr());
	shootshooter.WhileTrue(ShootShooter(&shooter, 3.0).ToPtr());
	shooterAngle.WhileTrue(ShooterAngle(&superStructure));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	std::string autoName = autoChooser.GetSelected();
	if (autoName == "None") {
		return  frc2::cmd::None();
	}

	return pathplanner::AutoBuilder::buildAuto(autoName);
}
