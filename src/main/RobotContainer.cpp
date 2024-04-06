// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h"
#include "OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h"
#include "main/Commands/ServoDashboard/ServoDashboard.h"
#include "main/Autos/AmpAutoCenterRace/AmpAutoCenterRace.h"
#include "main/Autos/SourceAutoCenterRace/SourceAutoCenterRace.h"

RobotContainer::RobotContainer() {
	pathplanner::NamedCommands::registerCommand("GroundGrabCommand", GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(3_s));
	pathplanner::NamedCommands::registerCommand("GroundGrabCommandLT", GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(10_s));
	pathplanner::NamedCommands::registerCommand("ClosedCommand", std::move(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionSpeakerCommand", std::move(frc2::cmd::Sequence(
		VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter, &targetProvider).ToPtr().WithTimeout(0.25_s),
		VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &storage).ToPtr()
	)));
	pathplanner::NamedCommands::registerCommand("VisionShootNoDelay", std::move(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &storage).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionAmpCommand", std::move(VisionAmpCommand(&superStructure, &shooter)));
	pathplanner::NamedCommands::registerCommand("StorageCommand", std::move(StorageCommand(&storage, 3_V).ToPtr()));
	pathplanner::NamedCommands::registerCommand("ShooterCommand", std::move(ShooterCommand(&shooter, 4.00).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionNoShoot", std::move(VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter, &targetProvider).ToPtr()));
	pathplanner::NamedCommands::registerCommand("AlignToNote", std::move(AlignToTrackedObject(&chassis, &noteTrackingCamera)));

	center7NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-7Notes");
	center5NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-5Notes");
	center4NoteAuto = pathplanner::AutoBuilder::buildAuto("CenterAuto-4Notes");
	ampAuto = pathplanner::AutoBuilder::buildAuto("AMPAuto");
	sourceAuto = pathplanner::AutoBuilder::buildAuto("SourceAuto");

	ampAutoCenterRace = AmpAutoCenterRace(&storage);
	sourceAutoCenterRace = SourceAutoCenterRace(&storage);

	autoChooser.SetDefaultOption("None, null, nada", defaultNoneAuto.get());
	autoChooser.AddOption("CenterAuto-7Notes", center7NoteAuto.get());
	autoChooser.AddOption("CenterAuto-5Notes", center5NoteAuto.get());
	autoChooser.AddOption("CenterAuto-4Notes", center4NoteAuto.get());
	autoChooser.AddOption("AMPAuto", ampAuto.get());
	autoChooser.AddOption("AMPAuto-Race", ampAutoCenterRace.get());
	autoChooser.AddOption("SourceAuto", sourceAuto.get());
	autoChooser.AddOption("SourceAuto-Race", sourceAutoCenterRace.get());

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

	ConfigureBindings();
	// ConfigureSysIdBindings(&chassis, &characterization);
}

void RobotContainer::ConfigureBindings() {
	noteOnStorage.WhileTrue(frc2::cmd::Sequence(
		BlinkEffect(&leds, "all", { 0, 255, 0 }, 0.25_s).ToPtr().WithTimeout(0.5_s),
		StaticEffect(&leds, "all", { 0, 255, 0 }).ToPtr()
	).IgnoringDisable(true));

	storageSensorEmergencyMode.WhileTrue(frc2::cmd::Sequence(
		BlinkEffect(&leds, "all", { 0, 0, 255 }, 0.25_s).ToPtr().WithTimeout(0.5_s),
		BlinkEffect(&leds, "all", { 0, 0, 125 }, 0.25_s).ToPtr().WithTimeout(0.5_s)
	).Repeatedly().IgnoringDisable(true));

	// shooterEmergencyMode.WhileTrue(frc2::cmd::Sequence(
	// 	BlinkEffect(&leds, "all", { 255, 255, 0 }, 0.25_s).ToPtr().WithTimeout(0.5_s),
	// 	BlinkEffect(&leds, "all", { 255, 0, 0 }, 0.25_s).ToPtr().WithTimeout(0.5_s)
	// ).Repeatedly().IgnoringDisable(true));

	// shooterEmergencyMode.OnTrue(frc2::cmd::Sequence(
	// 	frc2::cmd::RunOnce([&] {
	// 	driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
	// 	opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
	// }),
	// 	frc2::cmd::Wait(0.25_s),
	// 	frc2::cmd::RunOnce([&] {
	// 	driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
	// 	opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
	// }),
	// 	frc2::cmd::RunOnce([&] {
	// 	driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
	// 	opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
	// }),
	// 	frc2::cmd::Wait(0.25_s),
	// 	frc2::cmd::RunOnce([&] {
	// 	driver.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
	// 	opertr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
	// })
	// ));

	leds.SetDefaultCommand(BlinkEffect(&leds, "all", { 255, 0, 255 }, 1_s).IgnoringDisable(true));

	chassis.SetDefaultCommand(Drive(ChassisConstants::MaxModuleSpeed, &chassis, &driver));

	supportArms.SetDefaultCommand(FreeSupportArms(&supportArms, 50.00).Repeatedly());

	zeroHeading.OnTrue(ResetAngle(&chassis).ToPtr());

	//tabulate.ToggleOnTrue(TabulateCommand(&chassis, &superStructure, &shooter, &targetProvider).ToPtr());
	ampV.WhileTrue(VisionAmpCommand(&superStructure, &shooter));
	ampV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	speakerV.WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &opertr).ToPtr());
	speakerV.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	passNoteHigh.WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &targetProvider, &storage, PassNote::High).ToPtr());
	passNoteHigh.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	passNoteLow.WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &targetProvider, &storage, PassNote::Low).ToPtr());
	passNoteLow.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	//objectDetect.WhileTrue(AlignToTrackedObject(&chassis, &objectCamera));

	//Operator 
	ampM.WhileTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	ampM.OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	climbM.WhileTrue(ManualClimb(&chassis, &superStructure, &supportArms, &storage, &shooter, &opertr));
	climbM.OnFalse(
		frc2::cmd::Parallel(
			frc2::cmd::RunOnce([&] {
		chassis.setAcceptingVisionMeasurements(true);
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


	increaseUpperAngleOffset.OnTrue(
		BlinkEffect(&leds, "all", { 255, 0, 0 }, 0.05_s).WithTimeout(0.2_s)
	);

	increaseUpperAngleOffset.OnTrue(
		frc2::cmd::RunOnce([] {
			VisionSpeakerCommand::SetUpperAngleOffset(VisionSpeakerCommand::GetUpperAngleOffset() - 0.5);
		})
	);

	decreaseUpperAngleOffset.OnTrue(
		BlinkEffect(&leds, "all", {0, 255, 255 }, 0.05_s).WithTimeout(0.2_s)
	);

	decreaseUpperAngleOffset.OnTrue(
		frc2::cmd::RunOnce([] {
			VisionSpeakerCommand::SetUpperAngleOffset(VisionSpeakerCommand::GetUpperAngleOffset() + 0.5);
		})	
	);

	resetUpperAngleOffset.OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::RunOnce([] {
				VisionSpeakerCommand::ResetUpperAngleOffset();
			}),
			BlinkEffect(&leds, "all", {255, 255, 255 }, 0.05_s).WithTimeout(0.2_s)
		)
	);

	// closed.WhileTrue(ClosedCommand(&superStructure, &intake, &storage, &shooter).ToPtr());

	// shooterEmergencyStop.ToggleOnTrue(frc2::cmd::RunOnce([&] {
	// 	shooter.setEmergencyDisable(!shooter.isEmergencyDisabled());
	// }));

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
}


void RobotContainer::UpdateTelemetry() {
	superStructure.shuffleboardPeriodic();
	chassis.shuffleboardPeriodic();
	storage.shuffleboardPeriodic();
	intake.shuffleboardPeriodic();
	shooter.shuffleboardPeriodic();
}