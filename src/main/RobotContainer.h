// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <OvertureLib/Gamepad/Gamepad.h>
#include <OvertureLib/Robots/OverContainer/OverContainer.h>
#include <OvertureLib/Subsystems/LedsManager/LedsManager.h>
#include <OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Targeting/TargetProvider.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/SupportArms/SupportArms.h"

#include "Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "Commands/GroundGrabCommand/GroundGrabCommand.h"
#include "Commands/AmpCommand/AmpCommand.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/SpeakerCommand/SpeakerCommand.h"
#include "Commands/VisionAmpCommand/VisionAmpCommand.h"
#include "Commands/VisionSpeakerCommand/VisionSpeakerCommand.h"
#include "Commands/VisionSpeakerCommandNoShoot/VisionSpeakerCommandNoShoot.h"
#include "Commands/VisionSpeakerCommandPassNote/VisionSpeakerCommandPassNote.h"
#include "Commands/TabulateCommand/TabulateCommand.h"
#include "Commands/AlignToTrackedObject/AlignToTrackedObject.h"
#include "Commands/Climbing/Climbing.h"

#include "Helpers/ClosedLoopRotationHelper/ClosedLoopRotationHelper.h"

class RobotContainer : public OverContainer {
public:
	RobotContainer();

	// frc2::Command* GetAutonomousCommand();
	// frc2::CommandPtr GetTeleopResetCommand();
	void UpdateTelemetry();

private:
	void ConfigureBindings();
	void ConfigDriverBindings();
	void ConfigOperatorBindings();
	void ConfigDefaultCommands();
	void ConfigCharacterizationBindings();

	// Subsystems
	Intake intake;
	SuperStructure superStructure;
	Storage storage;
	Shooter shooter;
	// Chassis chassis;
	// SupportArms supportArms;

	// Helpers
	ClosedLoopRotationHelper rotationHelper;

	//Vision
#ifndef __FRC_ROBORIO__
	frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2024Crescendo);
#else
	frc::AprilTagFieldLayout tagLayout{ "/home/lvuser/deploy/tag_layout/7421-field.json" };
#endif
	//AprilTags shooterCamera{ &tagLayout, &chassis, {"Arducam_OV2311_USB_Camera", { {-14.950771_in, 0_m, 14.034697_m}, {-180_deg, -30_deg, 180_deg} }, 5_m, 9_m, 13_m} };
	// AprilTags frontRightSwerveModuleCamera{ &tagLayout, &chassis, {"Arducam_OV9281_USB_Camera", { {6.388283_in, -10.648092_in, 8.358231_in}, {0_deg, -28.125_deg, -30_deg} }} };
	// photon::PhotonCamera noteTrackingCamera{ "PSEye" };
	// TargetProvider targetProvider{ &tagLayout };

	// LedsManager leds{ 0, 240, {
	// 	{"all", {0, 239}}
	// } };

	// Controllers
	//frc::XboxController driver{ 0 };
	//frc::XboxController opertr{ 1 };

	Gamepad driverPad{ 0, 0.1, 0.2 };
	Gamepad operatorPad{ 1, 0.1, 0.2 };
	Gamepad characterizationPad{ 2, 0.1, 0.2 };

	// LED Triggers
	// frc2::Trigger noteOnStorage{ [this] {return storage.isNoteOnForwardSensor();} };
	// frc2::Trigger storageSensorEmergencyMode{ [this] {return !storage.isSensorAvailable();} };
	// frc2::Trigger intakeMotorActive{ [this] {return intake.getVoltage() != 0.0 && !storage.isNoteOnForwardSensor();} };

	//Autonomous
	/*frc2::CommandPtr defaultNoneAuto = frc2::cmd::None();
	frc2::CommandPtr center4NoteAuto = frc2::cmd::None();
	frc2::CommandPtr center5NoteAuto = frc2::cmd::None();
	frc2::CommandPtr center7NoteAuto = frc2::cmd::None();
	frc2::CommandPtr ampAuto = frc2::cmd::None();
	frc2::CommandPtr sourceAuto = frc2::cmd::None();

	frc2::CommandPtr ampAutoCenterRace = frc2::cmd::None();
	frc2::CommandPtr sourceAutoCenterRace = frc2::cmd::None();*/

	//Auto Chooser
	// frc::SendableChooser<frc2::Command*> autoChooser;
};
