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
	// ConfigureSysIdBindings(&chassis, &driver);
}

void RobotContainer::ConfigureBindings() {
	chassis.SetDefaultCommand(Drive(&chassis, &driver));

	// Configure the button bindings
	resetAngleButton.WhileTrue(ResetAngle(&chassis).ToPtr());
	climbButton.WhileTrue(AutoClimb(&chassis, &superStructure, &supportArms, &driver));
	// shootTrap.WhileTrue(TrapShoot(&chassis, &superStructure, &supportArms, &shooter, &storage));

	groundGrab.WhileTrue(GroundGrabCommand(&superStructure, &storage, &intake).ToPtr());
	groundGrab.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	sourceGrab.WhileTrue(SourceGrabCommand(&superStructure, &shooter).ToPtr());
	sourceGrab.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	ampShoot.WhileTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	ampShoot.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	speakerShoot.WhileTrue(SpeakerCommand(&superStructure, &shooter, &storage).ToPtr());
	speakerShoot.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	storageIn.WhileTrue(StorageCommand(&storage, 3_V).ToPtr());
	storageIn.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	ampButton.WhileTrue(VisionAmpCommand(&superStructure, &shooter, &storage));
	ampButton.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	sourceButton.WhileTrue(VisionSourceGrabCommand(&superStructure, &shooter, &storage));
	sourceButton.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	


	manualClimb.WhileTrue(ManualClimb(&chassis, &superStructure, &supportArms, &aprilTagCamera, &opertr));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	std::string autoName = autoChooser.GetSelected();
	if (autoName == "None") {
		return  frc2::cmd::None();
	}

	return pathplanner::AutoBuilder::buildAuto(autoName);
}
