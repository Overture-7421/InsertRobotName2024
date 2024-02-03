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
	autoChooser.AddOption("CenterAuto", "CenterAuto");
	autoChooser.AddOption("AMPAuto", "AMPAuto");
	autoChooser.AddOption("SourceAuto", "SourceAuto");
	autoChooser.AddOption("CenterAutoDirect", "CenterAutoDirect");

	pathplanner::NamedCommands::registerCommand("GroundGrabCommand", std::move(GroundGrabCommand(&superStructure, &storage, &intake).ToPtr()));
	pathplanner::NamedCommands::registerCommand("ClosedCommand", std::move(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr()));
	pathplanner::NamedCommands::registerCommand("SpeakerCommand", std::move(SpeakerCommand(&superStructure, &shooter, &storage).ToPtr()));
	pathplanner::NamedCommands::registerCommand("AmpCommand", std::move(AmpCommand(&superStructure, &shooter, &storage).ToPtr()));
	pathplanner::NamedCommands::registerCommand("StorageCommand", std::move(StorageCommand(&storage, 3_V).ToPtr()));
	pathplanner::NamedCommands::registerCommand("ShooterCommand", std::move(ShooterCommand(&shooter, 4.00).ToPtr()));

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
	ConfigureBindings();
}	

void RobotContainer::ConfigureBindings() {
	chassis.SetDefaultCommand(Drive(&chassis, &driver));

	// Configure the button bindings
	resetAngleButton.WhileTrue(ResetAngle(&chassis).ToPtr());
	climbButton.WhileTrue(Climb(&chassis, &superStructure, &supportArms, &driver));
	shootTrap.WhileTrue(TrapShoot(&chassis, &superStructure, &supportArms, &shooter, &storage));

	GroundGrab.WhileTrue(GroundGrabCommand(&superStructure, &storage, &intake).ToPtr());
	GroundGrab.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	SourceGrab.WhileTrue(SourceGrabCommand(&superStructure, &shooter).ToPtr());
	SourceGrab.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	AmpShoot.WhileTrue(AmpCommand(&superStructure, &shooter, &storage).ToPtr());
	AmpShoot.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	SpeakerShoot.WhileTrue(SpeakerCommand(&superStructure, &shooter, &storage).ToPtr());
	SpeakerShoot.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	std::string autoName = autoChooser.GetSelected();
	if (autoName == "None") {
		return  frc2::cmd::None();
	}

	return pathplanner::AutoBuilder::buildAuto(autoName);
}
