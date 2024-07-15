// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h"
#include "Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h"
#include "Commands/ServoDashboard/ServoDashboard.h"
#include "Autos/AmpAutoCenterRace/AmpAutoCenterRace.h"
#include "Autos/SourceAutoCenterRace/SourceAutoCenterRace.h"

RobotContainer::RobotContainer() {
	pathplanner::NamedCommands::registerCommand("GroundGrabCommand", GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(3_s));
	pathplanner::NamedCommands::registerCommand("GroundGrabCommandLT", GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(5_s));
	pathplanner::NamedCommands::registerCommand("GroundGrabCommandNT", GroundGrabCommand(&superStructure, &storage, &intake));

	pathplanner::NamedCommands::registerCommand("ClosedCommand", std::move(ClosedCommand(&superStructure, &intake, &storage, &shooter)));
	pathplanner::NamedCommands::registerCommand("VisionSpeakerCommand", std::move(frc2::cmd::Sequence(
		VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter, &targetProvider).ToPtr().WithTimeout(0.35_s),
		VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &storage).ToPtr()
	)));
	pathplanner::NamedCommands::registerCommand("VisionShootNoDelay", std::move(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &storage).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionAmpCommand", std::move(VisionAmpCommand(&superStructure, &shooter)));
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
	// autoChooser.AddOption("CenterAuto-4Notes", center4NoteAuto.get());
	// autoChooser.AddOption("AMPAuto", ampAuto.get());
	autoChooser.AddOption("AMPAuto-Race", ampAutoCenterRace.get());
	// autoChooser.AddOption("SourceAuto", sourceAuto.get());
	autoChooser.AddOption("SourceAuto-Race", sourceAutoCenterRace.get());
	// autoChooser.AddOption("Testing", testingAuto.get());


	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

	ConfigureBindings();
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


	intakeMotorActive.WhileTrue(
		BlinkEffect(&leds, "all", { 255, 0, 255 }, 0.1_s).ToPtr().Repeatedly()
	);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}

frc2::CommandPtr RobotContainer::GetTeleopResetCommand() {
	return frc2::cmd::Deadline(
		storage.storageCommand(StorageConstants::StopVolts),
		shooter.shooterCommand(ShooterConstants::StopSpeed),
		intake.intakeCommand(IntakeConstants::StopVolts)
	);
}

void RobotContainer::ConfigDriverBindings() {

	driverPad.rightStick(0.5).WhileTrue(frc2::cmd::Run([&] {
		chassis.setRotationClosedLoop(
			units::meters_per_second_t{ Utils::ApplyAxisFilter(-driverPad.GetLeftY()) * ChassisConstants::MaxModuleSpeed },
			units::meters_per_second_t{ Utils::ApplyAxisFilter(-driverPad.GetLeftX()) * ChassisConstants::MaxModuleSpeed },
			driverPad.getRightStickDirection()
		);
	}, { &chassis }).BeforeStarting([&] {
		chassis.setHeadingOverride(true);
		chassis.headingController.SetConstraints({ 2_rad_per_s, 1.5_rad_per_s_sq });
	}).FinallyDo([&] {
		chassis.setHeadingOverride(false);
		chassis.headingController.SetConstraints({ 18_rad_per_s, 18_rad_per_s_sq * 2 });
	}));

	driverPad.leftBumperOnly().WhileTrue(VisionAmpCommand(&superStructure, &shooter));
	driverPad.leftBumperOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.rightBumperOnly().WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &operatorPad).ToPtr());
	driverPad.rightBumperOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.Back().OnTrue(chassis.resetHeading());

	driverPad.Y().WhileTrue(AutoClimb(&chassis, &superStructure, &supportArms, &storage, &shooter, &operatorPad));
	driverPad.Y().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.X().WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &targetProvider, &storage, PassNote::High).ToPtr());
	driverPad.X().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.B().WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &targetProvider, &storage, PassNote::Low).ToPtr());
	driverPad.B().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.A().WhileTrue(AlignToTrackedObject(&chassis, &noteTrackingCamera));

	//tabulate.ToggleOnTrue(TabulateCommand(&chassis, &superStructure, &shooter, &targetProvider).ToPtr());

	// tabulate.OnTrue(frc2::cmd::RunOnce([&] {
	// 	chassis.setHeadingOverride(true);
	// 	chassis.setTargetHeading({ -90_deg });
	// }));

	// tabulate.OnFalse(frc2::cmd::RunOnce([&] {
	// 	chassis.setHeadingOverride(false);
	// }));
}

void RobotContainer::ConfigOperatorBindings() {

	operatorPad.leftBumperOnly().WhileTrue(AmpCommand(&superStructure, &shooter));
	operatorPad.leftBumperOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	operatorPad.B().OnTrue(frc2::cmd::Parallel(
		storage.storageCommand(StorageConstants::SpitVolts),
		intake.intakeCommand(IntakeConstants::ReverseVolts)
	));
	operatorPad.B().OnFalse(frc2::cmd::Parallel(
		storage.storageCommand(StorageConstants::SpitVolts),
		intake.intakeCommand(IntakeConstants::ReverseVolts)
	));

	operatorPad.Y().WhileTrue(ManualClimb(&chassis, &superStructure, &supportArms, &storage, &shooter, &operatorPad));
	operatorPad.Y().OnFalse(frc2::cmd::Parallel(
		frc2::cmd::RunOnce([&] {chassis.setAcceptingVisionMeasurements(true);}),
		ClosedCommand(&superStructure, &intake, &storage, &shooter)
	));

	operatorPad.leftTriggerOnly().WhileTrue(storage.storageCommand(StorageConstants::ScoreVolts));
	operatorPad.leftTriggerOnly().OnFalse(storage.storageCommand(StorageConstants::StopVolts));

	operatorPad.rightBumperOnly().WhileTrue(SpeakerCommand(&superStructure, &shooter));
	operatorPad.rightBumperOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	operatorPad.A().WhileTrue(GroundGrabCommand(&superStructure, &storage, &intake, true));
	operatorPad.A().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));
	operatorPad.rightTriggerOnly().WhileTrue(GroundGrabCommand(&superStructure, &storage, &intake, true));
	operatorPad.rightTriggerOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	operatorPad.upDpad().OnTrue(
		BlinkEffect(&leds, "all", { 255, 0, 0 }, 0.05_s).WithTimeout(0.2_s)
	);

	operatorPad.upDpad().OnTrue(
		frc2::cmd::RunOnce([] {
		VisionSpeakerCommand::SetUpperAngleOffset(VisionSpeakerCommand::GetUpperAngleOffset() - 0.5);
	})
	);

	operatorPad.downDpad().OnTrue(
		BlinkEffect(&leds, "all", { 0, 255, 255 }, 0.05_s).WithTimeout(0.2_s)
	);

	operatorPad.downDpad().OnTrue(
		frc2::cmd::RunOnce([] {
		VisionSpeakerCommand::SetUpperAngleOffset(VisionSpeakerCommand::GetUpperAngleOffset() + 0.5);
	})
	);

	operatorPad.leftDpad().OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::RunOnce([] {
		VisionSpeakerCommand::ResetUpperAngleOffset();
	}),
			BlinkEffect(&leds, "all", { 255, 255, 255 }, 0.05_s).WithTimeout(0.2_s)
		)
	);

	operatorPad.rightDpad().OnTrue(superStructure.superStructureCommand({ 90, 0 }));
	operatorPad.rightDpad().OnFalse(superStructure.superStructureCommand(SuperStructureConstants::GroundGrabState));

}

void RobotContainer::ConfigDefaultCommands() {

	leds.SetDefaultCommand(BlinkEffect(&leds, "all", { 255, 0, 255 }, 1_s).IgnoringDisable(true));

	chassis.SetDefaultCommand(frc2::cmd::Run([&] {
		chassis.setFieldRelative(
			units::meters_per_second_t{ Utils::ApplyAxisFilter(driverPad.GetLeftY()) * ChassisConstants::MaxModuleSpeed },
			units::meters_per_second_t{ Utils::ApplyAxisFilter(driverPad.GetLeftX()) * ChassisConstants::MaxModuleSpeed },
			units::radians_per_second_t{ Utils::ApplyAxisFilter(-driverPad.getTwist()) * ChassisConstants::MaxAngularSpeed }
		);
	}, { &chassis }).BeforeStarting([&] {
		chassis.setAlliance();
	}));

	supportArms.SetDefaultCommand(supportArms.freeArmsCommand(25.00).Repeatedly());
}


void RobotContainer::UpdateTelemetry() {
	superStructure.shuffleboardPeriodic();
	chassis.shuffleboardPeriodic();
	storage.shuffleboardPeriodic();
	intake.shuffleboardPeriodic();
	shooter.shuffleboardPeriodic();
}