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

	// Configure the button bindings
	resetAngleButton.WhileTrue(ResetAngle(&chassis).ToPtr());


	intakeTrue.OnTrue(IntakeCommand(&intake, 3_V).ToPtr());
	intakeTrue.OnFalse(IntakeCommand(&intake, 0_V).ToPtr());
	storageTrue.OnTrue(StorageCommand(&storage, 2_V).ToPtr());
	storageTrue.OnFalse(StorageCommand(&storage, 0_V).ToPtr());
	storageFalse.OnTrue(StorageCommand(&storage, -2_V).ToPtr());
	storageFalse.OnFalse(StorageCommand(&storage, 0_V).ToPtr());
	superStructurePos1.WhileTrue(SuperStructureCommand(&superStructure, {0.0, 0.0}).ToPtr());
	superStructurePos2.WhileTrue(SuperStructureCommand(&superStructure, {30.0, -30.0}).ToPtr());
	supportArmOpen.OnTrue(SupportArmCommand2(&supportArms, {-90.0}).ToPtr())
					.OnFalse(SupportArmCommand2(&supportArms, {0.0}).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	std::string autoName = autoChooser.GetSelected();
	if (autoName == "None") {
		return  frc2::cmd::None();
	}

	return pathplanner::AutoBuilder::buildAuto(autoName);
}
