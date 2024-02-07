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
	// ConfigureSysIdBindings(&chassis, &driver);
}

void RobotContainer::ConfigureBindings() {
	chassis.SetDefaultCommand(Drive(&chassis, &driver));

	ampV.WhileTrue(VisionAmpCommand(&superStructure, &shooter, &storage));
	ampV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());
	
	sourceV.WhileTrue(VisionSourceGrabCommand(&superStructure, &shooter, &storage));
	sourceV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	speakerV.WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &opertr).ToPtr());
	speakerV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// Operator 
	ampM.WhileTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	ampM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	sourceM.WhileTrue(SourceGrabCommand(&superStructure, &shooter).ToPtr());
	sourceM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	climbV.WhileTrue(AutoClimb(&chassis, &superStructure, &supportArms, &opertr));
	climbV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	climbM.WhileTrue(ManualClimb(&chassis, &superStructure, &supportArms, &aprilTagCamera, &opertr));
	climbM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	shootM.WhileTrue(StorageCommand(&storage, 3_V).ToPtr());
	shootM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	speakerM.WhileTrue(SpeakerCommand(&superStructure, &shooter).ToPtr());
	speakerM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	trapV.WhileTrue(TrapShoot(&chassis, &superStructure, &supportArms, &shooter, &storage));
	trapV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	closed.WhileTrue(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	intakeM.WhileTrue(IntakeCommand(&intake, 4_V).ToPtr());
	intakeM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	std::string autoName = autoChooser.GetSelected();
	if (autoName == "None") {
		return  frc2::cmd::None();
	}

	return pathplanner::AutoBuilder::buildAuto(autoName);
}
