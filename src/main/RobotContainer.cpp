// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h"
#include "OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h"

RobotContainer::RobotContainer() {

	pathplanner::NamedCommands::registerCommand("GroundGrabCommand", GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(3_s));
	pathplanner::NamedCommands::registerCommand("ClosedCommand", std::move(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionSpeakerCommand", std::move(frc2::cmd::Sequence(
		VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter).ToPtr().WithTimeout(0.1_s),
		VisionSpeakerCommand(&chassis, &superStructure, &shooter, &storage).ToPtr()
	)));
	pathplanner::NamedCommands::registerCommand("VisionAmpCommand", std::move(VisionAmpCommand(&superStructure, &shooter, &storage)));
	pathplanner::NamedCommands::registerCommand("StorageCommand", std::move(StorageCommand(&storage, 3_V).ToPtr()));
	pathplanner::NamedCommands::registerCommand("ShooterCommand", std::move(ShooterCommand(&shooter, 4.00).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionNoShoot", std::move(VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter).ToPtr()));

	center7NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-7Notes");
	center4NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-4Notes");
	ampAuto = pathplanner::AutoBuilder::buildAuto("AMPAuto");
	sourceAuto = pathplanner::AutoBuilder::buildAuto("SourceAuto");

	autoChooser.SetDefaultOption("None, null, nada", defaultNoneAuto.get());
	autoChooser.AddOption("CenterAuto-7Notes", center7NoteAuto.get());
	autoChooser.AddOption("CenterAuto-4Notes", center4NoteAuto.get());
	autoChooser.AddOption("AMPAuto", ampAuto.get());
	autoChooser.AddOption("SourceAuto", sourceAuto.get());

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

	ConfigureBindings();
	// ConfigureSysIdBindings(&chassis, &characterization);
}

void RobotContainer::ConfigureBindings() {
	noteOnStorage.WhileTrue(frc2::cmd::Sequence(
		BlinkEffect(&leds, "all", { 0, 255, 0 }, 0.25_s).ToPtr().WithTimeout(0.5_s),
		StaticEffect(&leds, "all", { 0, 255, 0 }).ToPtr()
	));

	leds.SetDefaultCommand(BlinkEffect(&leds, "all", { 255, 0, 255 }, 1_s));

	chassis.SetDefaultCommand(Drive(&chassis, &driver));

	supportArms.SetDefaultCommand(FreeSupportArms(&supportArms, -10.00).Repeatedly());
	// shooter.SetDefaultCommand(ShooterDefaultCommand(&chassis, &shooter));

	// tabulate.ToggleOnTrue(TabulateCommand(&chassis, &superStructure, &shooter).ToPtr());
	// tabulate.OnTrue(SuperStructureCommand(&superStructure, { 0, -90 }).ToPtr());
	// tabulate.OnFalse(SuperStructureCommand(&superStructure, { 0, 0 }).ToPtr());

	zeroHeading.OnTrue(ResetAngle(&chassis).ToPtr());

	ampV.WhileTrue(VisionAmpCommand(&superStructure, &shooter, &storage));
	ampV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	speakerV.WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &opertr).ToPtr());
	speakerV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// Operator 
	ampM.WhileTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	ampM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	climbM.WhileTrue(ManualClimb(&chassis, &superStructure, &supportArms, &aprilTagCamera, &storage, &shooter, &opertr));
	climbM.OnFalse(
		frc2::cmd::Parallel(
			frc2::cmd::RunOnce([&] {
		aprilTagCamera.setPoseEstimator(true);
	}),
			ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr()
		)
	);

	climbV.WhileTrue(AutoClimb(&chassis, &superStructure, &supportArms, &storage, &shooter, &opertr));
	climbV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	shootM.WhileTrue(StorageCommand(&storage, StorageConstants::SpeakerScoreVolts).ToPtr());
	shootM.OnFalse(StorageCommand(&storage, 0_V).ToPtr());

	spitM.OnTrue(frc2::cmd::Parallel(
		StorageCommand(&storage, -5_V).ToPtr(),
		IntakeCommand(&intake, -8_V).ToPtr()
	));
	spitM.OnFalse(frc2::cmd::Parallel(
		StorageCommand(&storage, 0_V).ToPtr(),
		IntakeCommand(&intake, 0_V).ToPtr()
	));

	speakerM.WhileTrue(SpeakerCommand(&superStructure, &shooter).ToPtr());
	speakerM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	closed.WhileTrue(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	intakeM.WhileTrue(GroundGrabCommand(&superStructure, &storage, &intake));
	intakeM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}

frc2::CommandPtr RobotContainer::GetTeleopResetCommand() {
	return frc2::cmd::Deadline(
		StorageCommand(&storage, 0_V).ToPtr(),
		ShooterCommand(&shooter, 0).ToPtr(),
		IntakeCommand(&intake, 0_V).ToPtr()
	);
	// return frc2::cmd::None();
}


void RobotContainer::UpdateTelemetry() {
	superStructure.shuffleboardPeriodic();
	chassis.shuffleboardPeriodic();
	storage.shuffleboardPeriodic();
	shooter.shuffleboardPeriodic();
}
