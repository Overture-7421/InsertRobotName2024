// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h>
#include "Commands/ServoDashboard/ServoDashboard.h"
#include "Autos/AmpAutoCenterRace/AmpAutoCenterRace.h"
#include "Autos/SourceAutoCenterRace/SourceAutoCenterRace.h"
#include "Subsystems/SupportArms/Constants.h"

RobotContainer::RobotContainer() {
	pathplanner::NamedCommands::registerCommand("GroundGrabCommand", std::move(GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(3_s)));
	pathplanner::NamedCommands::registerCommand("GroundGrabCommandLT", std::move(GroundGrabCommand(&superStructure, &storage, &intake).WithTimeout(5_s)));
	pathplanner::NamedCommands::registerCommand("GroundGrabCommandNT", std::move(GroundGrabCommand(&superStructure, &storage, &intake)));

	pathplanner::NamedCommands::registerCommand("ClosedCommand", std::move(ClosedCommand(&superStructure, &intake, &storage, &shooter)));
	pathplanner::NamedCommands::registerCommand("VisionSpeakerCommand", std::move(frc2::cmd::Sequence(
		VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter, &targetProvider).ToPtr().WithTimeout(0.35_s),
		VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &storage).ToPtr()
	)));
	pathplanner::NamedCommands::registerCommand("VisionShootNoDelay", std::move(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &storage).ToPtr()));
	pathplanner::NamedCommands::registerCommand("VisionNoShoot", std::move(VisionSpeakerCommandNoShoot(&chassis, &superStructure, &shooter, &targetProvider).ToPtr()));
	pathplanner::NamedCommands::registerCommand("AlignToNote", std::move(AlignToTrackedObject(&chassis, &noteTrackingCamera, &alignHelper)));


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

	chassis.setAcceptingVisionMeasurements(true);

	ConfigureBindings();
	ConfigDriverBindings();
	ConfigOperatorBindings();
	ConfigDefaultCommands();
	ConfigCharacterizationBindings();
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
	driverPad.rightStick(0.2).WhileTrue(frc2::cmd::Run([&] {
		rotationHelper.setTargetAngle(
			frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? driverPad.getRightStickDirection().RotateBy({ 180_deg }) : driverPad.getRightStickDirection()
		);
	}).BeforeStarting([&] {
		chassis.enableSpeedHelper(&rotationHelper);
	}).FinallyDo([&] {
		chassis.disableSpeedHelper();
	}));

	driverPad.leftBumperOnly().WhileTrue(VisionAmpCommand(&superStructure, &shooter));
	driverPad.leftBumperOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.rightBumperOnly().WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &targetProvider, &operatorPad).ToPtr());
	driverPad.rightBumperOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.Back().OnTrue(frc2::cmd::Either(
		frc2::cmd::RunOnce([&] { chassis.resetHeading(180); }),
		frc2::cmd::RunOnce([&] { chassis.resetHeading(); }),
		[&] {return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;}
	));

	driverPad.X().WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &targetProvider, &storage, PassNote::High).ToPtr());
	driverPad.X().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.B().WhileTrue(VisionSpeakerCommandPassNote(&chassis, &superStructure, &shooter, &targetProvider, &storage, PassNote::Low).ToPtr());
	driverPad.B().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	driverPad.A().WhileTrue(AlignToTrackedObject(&chassis, &noteTrackingCamera, &alignHelper));

	driverPad.Y().WhileTrue(AutoClimb(&chassis, &superStructure, &supportArms, &storage, &shooter, &operatorPad));
	driverPad.Y().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	//driverPad.Y().ToggleOnTrue(TabulateCommand(&chassis, &superStructure, &shooter, &targetProvider).ToPtr());

}

void RobotContainer::ConfigOperatorBindings() {

	operatorPad.leftBumperOnly().WhileTrue(AmpCommand(&superStructure, &shooter));
	operatorPad.leftBumperOnly().OnFalse(ClosedCommand(&superStructure, &intake, &storage, &shooter));

	operatorPad.B().OnTrue(frc2::cmd::Sequence(
		superStructure.superStructureCommand(SuperStructureConstants::GroundGrabState),
		storage.storageCommand(StorageConstants::SpitVolts),
		intake.intakeCommand(IntakeConstants::ReverseVolts)
	));
	operatorPad.B().OnFalse(frc2::cmd::Sequence(
		superStructure.superStructureCommand(SuperStructureConstants::ClosedState),
		storage.storageCommand(StorageConstants::StopVolts),
		intake.intakeCommand(IntakeConstants::StopVolts)
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
	operatorPad.rightTriggerOnly().WhileTrue(GroundGrabCommand(&superStructure, &storage, &intake));
	operatorPad.rightTriggerOnly().OnFalse(
		frc2::cmd::Parallel(
			storage.storageCommand(StorageConstants::StopVolts),
			intake.intakeCommand(IntakeConstants::StopVolts)
		)
	);

	operatorPad.upDpad().OnTrue(frc2::cmd::Parallel(
		frc2::cmd::RunOnce([] {
		VisionSpeakerCommand::SetUpperAngleOffset(VisionSpeakerCommand::GetUpperAngleOffset() - 0.5);
	}),
		BlinkEffect(&leds, "all", { 0, 255, 255 }, 0.05_s).WithTimeout(0.2_s)
	));

	operatorPad.downDpad().OnTrue(frc2::cmd::Parallel(
		frc2::cmd::RunOnce([] {
		VisionSpeakerCommand::SetUpperAngleOffset(VisionSpeakerCommand::GetUpperAngleOffset() + 0.5);
	}),
		BlinkEffect(&leds, "all", { 255, 0, 0 }, 0.05_s).WithTimeout(0.2_s)
	));

	operatorPad.leftDpad().OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::RunOnce([] {
		VisionSpeakerCommand::ResetUpperAngleOffset();
	}),
			BlinkEffect(&leds, "all", { 255, 255, 255 }, 0.05_s).WithTimeout(0.2_s)
		)
	);

	operatorPad.rightDpad().OnTrue(superStructure.superStructureCommand({ 90, 0 }));
	operatorPad.rightDpad().OnFalse(superStructure.superStructureCommand(SuperStructureConstants::ClosedState));
}

void RobotContainer::ConfigDefaultCommands() {
	leds.SetDefaultCommand(BlinkEffect(&leds, "all", { 255, 0, 255 }, 1_s).IgnoringDisable(true));

	chassis.SetDefaultCommand(frc2::cmd::Run([&] {
		chassis.driveFieldRelative(
			{
				filterX.Calculate(units::meters_per_second_t{ Utils::ApplyAxisFilter(-driverPad.GetLeftY()) * ChassisConstants::MaxModuleSpeed.value() }),
				filterY.Calculate(units::meters_per_second_t{ Utils::ApplyAxisFilter(-driverPad.GetLeftX()) * ChassisConstants::MaxModuleSpeed.value() }),
				units::radians_per_second_t{ Utils::ApplyAxisFilter(-driverPad.getTwist()) * ChassisConstants::MaxAngularSpeed.value() }
			}
		);
	}, { &chassis }).BeforeStarting(
		frc2::cmd::RunOnce([&] {
		chassis.setAllianceColor();
	})));

	supportArms.SetDefaultCommand(supportArms.freeArmsCommand(SupportArmsConstants::IdleAngle).Repeatedly());
	//supportArms.SetDefaultCommand(ServoDashboard(&supportArms));

}

void RobotContainer::ConfigCharacterizationBindings() {
	//characterizationPad.A().ToggleOnTrue(TabulateCommand(&chassis, &superStructure, &shooter, &targetProvider).ToPtr());
}

void RobotContainer::UpdateTelemetry() {
	superStructure.shuffleboardPeriodic();
	accelFFSuperStructure.shuffleboardPeriodic();
	chassis.shuffleboardPeriodic();
	storage.shuffleboardPeriodic();
	intake.shuffleboardPeriodic();
	shooter.shuffleboardPeriodic();
}