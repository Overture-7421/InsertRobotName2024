// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <iostream>

RobotContainer::RobotContainer() {
	autoChooser.SetDefaultOption("None, null, nada", "None");
	autoChooser.AddOption("MiddleNote", "MiddleNote");

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

	ConfigureBindings();
	// ConfigureSysIdBindings(&chassis, &driver);
}

void RobotContainer::ConfigureBindings()
{	
	// SuperStructure Upper
	// characterization.B().WhileTrue(superStructure.sysIdQuasistaticUpper(frc2::sysid::Direction::kForward));
	// characterization.A().WhileTrue(superStructure.sysIdQuasistaticUpper(frc2::sysid::Direction::kReverse));

	// characterization.X().WhileTrue(superStructure.sysIdDynamicUpper(frc2::sysid::Direction::kForward));
	// characterization.Y().WhileTrue(superStructure.sysIdDynamicUpper(frc2::sysid::Direction::kReverse));

	// Shooter
	// characterization.B().WhileTrue(shooter.sysIdQuasistatic(frc2::sysid::Direction::kForward));
	// characterization.A().WhileTrue(shooter.sysIdQuasistatic(frc2::sysid::Direction::kReverse));

	// characterization.X().WhileTrue(shooter.sysIdDynamic(frc2::sysid::Direction::kForward));
	// characterization.Y().WhileTrue(shooter.sysIdDynamic(frc2::sysid::Direction::kReverse));

	// chassis.SetDefaultCommand(Drive(&chassis, &driver));

	// ampV.WhileTrue(VisionAmpCommand(&superStructure, &shooter, &storage));
	// ampV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());
	
	// sourceV.WhileTrue(VisionSourceGrabCommand(&superStructure, &shooter, &storage));
	// sourceV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// speakerV.WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &opertr).ToPtr());
	// speakerV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// // Operator 
	// ampM.WhileTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	// ampM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// sourceM.WhileTrue(SourceGrabCommand(&superStructure, &shooter, &storage).ToPtr());
	// sourceM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// climbV.WhileTrue(AutoClimb(&chassis, &superStructure, &supportArms, &opertr));
	// climbV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// climbM.WhileTrue(ManualClimb(&chassis, &superStructure, &supportArms, &aprilTagCamera, &opertr));
	// climbM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// shootM.WhileTrue(StorageCommand(&storage, 3_V).ToPtr());
	// shootM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// speakerM.WhileTrue(SpeakerCommand(&superStructure, &shooter).ToPtr());
	// speakerM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// trapV.WhileTrue(TrapShoot(&chassis, &superStructure, &supportArms, &shooter, &storage));
	// trapV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// closed.WhileTrue(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// intakeM.WhileTrue(GroundGrabCommand(&superStructure, &storage, &intake).ToPtr());
	// intakeM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	std::string autoName = autoChooser.GetSelected();
	if (autoName == "None")
	{
		return frc2::cmd::None();
	}

	return pathplanner::AutoBuilder::buildAuto(autoName);
}
