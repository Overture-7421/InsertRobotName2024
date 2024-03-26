// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h"
#include "OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h"
#include "main/Commands/ServoDashboard/ServoDashboard.h"

RobotContainer::RobotContainer() {
	pathplanner::NamedCommands::registerCommand("GroundGrabCommand", GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(3_s));
	pathplanner::NamedCommands::registerCommand("ClosedCommand", std::move(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionSpeakerCommand", std::move(frc2::cmd::Sequence(
		VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter, &aprilTagCamera).ToPtr().WithTimeout(0.1_s),
		VisionSpeakerCommand(&chassis, &superStructure, &shooter, &aprilTagCamera, &storage).ToPtr()
	)));
	pathplanner::NamedCommands::registerCommand("VisionShootNoDelay", std::move(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &aprilTagCamera, &storage).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionAmpCommand", std::move(VisionAmpCommand(&superStructure, &shooter)));
	pathplanner::NamedCommands::registerCommand("StorageCommand", std::move(StorageCommand(&storage, 3_V).ToPtr()));
	pathplanner::NamedCommands::registerCommand("ShooterCommand", std::move(ShooterCommand(&shooter, 4.00).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionNoShoot", std::move(VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter, &aprilTagCamera).ToPtr()));

	center7NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-7Notes");
	center5NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-5Notes");
	center4NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-4Notes");
	ampAuto = pathplanner::AutoBuilder::buildAuto("AMPAuto");
	sourceAuto = pathplanner::AutoBuilder::buildAuto("SourceAuto");


	ampAutoCenterRate = frc2::cmd::Sequence(
		pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
		frc2::cmd::Parallel(
			pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto1")),
			pathplanner::NamedCommands::getCommand("GroundGrabCommand")
		),
		//Go back to shoot or grab next note if stolen
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Deadline(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto2")),
					pathplanner::NamedCommands::getCommand("VisionNoShoot")
				),
				pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto3")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			frc2::cmd::Sequence(
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("RaceAmpAuto_Stolen1")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			[&] {return storage.isNoteOnForwardSensor();}
		),
		//Go back to shoot or grab next note if stolen
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Deadline(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto4")),
					pathplanner::NamedCommands::getCommand("VisionNoShoot")
				),
				pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto5")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			frc2::cmd::Sequence(
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("RaceAmpAuto_Stolen2")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			[&] {return storage.isNoteOnForwardSensor();}
		),
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto6")),
			pathplanner::NamedCommands::getCommand("VisionNoShoot")
		),
		pathplanner::NamedCommands::getCommand("VisionSpeakerCommand")
	);


	autoChooser.SetDefaultOption("None, null, nada", defaultNoneAuto.get());
	autoChooser.AddOption("CenterAuto-7Notes", center7NoteAuto.get());
	autoChooser.AddOption("CenterAuto-5Notes", center5NoteAuto.get());
	autoChooser.AddOption("CenterAuto-4Notes", center4NoteAuto.get());
	autoChooser.AddOption("AMPAuto", ampAuto.get());
	autoChooser.AddOption("AMPAuto-Race", ampAutoCenterRate.get());
	autoChooser.AddOption("SourceAuto", sourceAuto.get());

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

	ConfigureBindings();
	// ConfigureSysIdBindings(&chassis, &characterization);
}

void RobotContainer::ConfigureBindings() {
	// characterization.A().WhileTrue(superStructure.sysIdQuasistaticLower(frc2::sysid::Direction::kForward));
	// characterization.B().WhileTrue(superStructure.sysIdQuasistaticLower(frc2::sysid::Direction::kReverse));
	// characterization.Y().WhileTrue(superStructure.sysIdDynamicLower(frc2::sysid::Direction::kForward));
	// characterization.X().WhileTrue(superStructure.sysIdDynamicLower(frc2::sysid::Direction::kReverse));

	
	noteOnStorage.WhileTrue(frc2::cmd::Sequence(
		BlinkEffect(&leds, "all", { 0, 255, 0 }, 0.25_s).ToPtr().WithTimeout(0.5_s),
		StaticEffect(&leds, "all", { 0, 255, 0 }).ToPtr()
	));

	shooterEmergencyMode.WhileTrue(frc2::cmd::Sequence(
		BlinkEffect(&leds, "all", { 255, 255, 0 }, 0.25_s).ToPtr().WithTimeout(0.5_s),
		BlinkEffect(&leds, "all", { 255, 0, 0 }, 0.25_s).ToPtr().WithTimeout(0.5_s)
	).Repeatedly());

	shooterEmergencyMode.OnTrue(frc2::cmd::Sequence(
		frc2::cmd::RunOnce([&] {
		driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
		opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
	}),
		frc2::cmd::Wait(0.25_s),
		frc2::cmd::RunOnce([&] {
		driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
		opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
	}),
		frc2::cmd::RunOnce([&] {
		driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
		opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
	}),
		frc2::cmd::Wait(0.25_s),
		frc2::cmd::RunOnce([&] {
		driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
		opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
	})
	));

	leds.SetDefaultCommand(BlinkEffect(&leds, "all", { 255, 0, 255 }, 1_s));

	chassis.SetDefaultCommand(Drive(ChassisConstants::MaxModuleSpeed, &chassis, &driver));

	supportArms.SetDefaultCommand(FreeSupportArms(&supportArms, 50.00).Repeatedly());

	zeroHeading.OnTrue(ResetAngle(&chassis).ToPtr());

	ampV.WhileTrue(VisionAmpCommand(&superStructure, &shooter));
	ampV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	speakerV.WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &aprilTagCamera, &opertr).ToPtr());
	speakerV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	passNoteHigh.WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &aprilTagCamera.GetAprilTagLayout(), &storage, PassNote::High).ToPtr());
	passNoteHigh.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	passNoteLow.WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &aprilTagCamera.GetAprilTagLayout(), &storage, PassNote::Low).ToPtr());
	passNoteLow.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

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

	ampM.WhileTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	ampM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());


	ampM.WhileTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	ampM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

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

	shooterEmergencyStop.ToggleOnTrue(frc2::cmd::RunOnce([&] {
		shooter.setEmergencyDisable(!shooter.isEmergencyDisabled());
	}));

	manualFrontalClimb.OnTrue(SuperStructureCommand(&superStructure, { 90, 0 }).ToPtr());
	manualFrontalClimb.OnFalse(SuperStructureCommand(&superStructure, SuperStructureConstants::GroundGrabState).ToPtr());

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
	intake.shuffleboardPeriodic();
	shooter.shuffleboardPeriodic();
}
