// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Targeting/TargetProvider.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/SupportArms/SupportArms.h"
#include "Commands/ResetAngle/ResetAngle.h"

#include "Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"

#include "Commands/GroundGrabCommand/GroundGrabCommand.h"
#include "Commands/AmpCommand/AmpCommand.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/ClosedCommandSmooth/ClosedCommandSmooth.h"
#include "Commands/SpeakerCommand/SpeakerCommand.h"
#include "Commands/VisionAmpCommand/VisionAmpCommand.h"
#include "Commands/VisionSpeakerCommand/VisionSpeakerCommand.h"
#include "Commands/VisionSpeakerCommandNoShoot/VisionSpeakerCommandNoShoot.h"
#include "Commands/ResetAngle/ResetAngle.h"
#include "Commands/ShooterDefaultCommand/ShooterDefaultCommand.h"
#include "Commands/FreeSupportArms/FreeSupportArms.h"
#include "Commands/VisionSpeakerCommandPassNote/VisionSpeakerCommandPassNote.h"

#include "Commands/TabulateCommand/TabulateCommand.h"
#include "Commands/AlignToTrackedObject/AlignToTrackedObject.h"


#include "Commands/Climbing/Climbing.h"
#include "Commands/TrapShoot/TrapShoot.h"

#include "Commands/Drive/Drive.h"
#include "Characterization/SysIDRoutineBot.h"
#include "Subsystems/LedsManager/LedsManager.h"
#include "Subsystems/Vision/AprilTags/AprilTags.h"

class RobotContainer : public SysIDRoutineBot {
public:
	RobotContainer();

	frc2::Command* GetAutonomousCommand();
	frc2::CommandPtr GetTeleopResetCommand();
	void UpdateTelemetry();

private:
	void ConfigureBindings();

	// Subsystems
	Intake intake;
	SuperStructure superStructure;
	Storage storage;
	Shooter shooter;
	Chassis chassis;
	SupportArms supportArms;

	//Vision
	frc::AprilTagFieldLayout tagLayout{ "/home/lvuser/deploy/tag_layout/7421-field.json" };
	AprilTags shooterCamera{ &tagLayout, &chassis, {"Arducam_OV2311_USB_Camera", { {-0.3686515106_m, 0_m, 0.3518230454_m}, {-180_deg, -23_deg, 180_deg} }, 5_m, 9_m, 13_m} };
	AprilTags frontRightSwerveModuleCamera{ &tagLayout, &chassis, {"Arducam_OV9281_USB_Camera", { {6.433997_in, -10.746927_in, 8.52786_in}, {0_deg, -28.125_deg, -30_deg} }} };
	photon::PhotonCamera noteTrackingCamera{ "PSEye" };
	TargetProvider targetProvider{ &tagLayout };

	LedsManager leds{ 0, 240, {
		{"all", {0, 239}}
	} };

	// Controllers
	frc::XboxController driver{ 0 };
	frc::XboxController opertr{ 1 };

	// frc2::CommandXboxController characterization {5};

	// Driver Commands
	frc2::Trigger ampV{ [this] {return driver.GetLeftTriggerAxis() > 0.1;} };
	frc2::Trigger sourceV{ [this] {return driver.GetRightBumper();} };
	frc2::Trigger speakerV{ [this] {return driver.GetRightTriggerAxis() > 0.1;} };	// TO GET TESTED
	frc2::Trigger zeroHeading{ [this] {return driver.GetBackButton();} };
	frc2::Trigger climbV{ [this] {return driver.GetYButton();} };
	frc2::Trigger passNoteHigh{ [this] {return driver.GetXButton();} };
	frc2::Trigger passNoteLow{ [this] {return driver.GetBButton();} };
	frc2::Trigger tabulate{ [this] {return driver.GetAButton();} };

	// Mechanism Commands
	frc2::Trigger ampM{ [this] {return opertr.GetLeftBumper();} };
	frc2::Trigger spitM{ [this] {return opertr.GetBButton();} };
	frc2::Trigger climbM{ [this] {return opertr.GetYButton();} };
	frc2::Trigger shootM{ [this] {return opertr.GetLeftTriggerAxis() > 0.1;} };
	frc2::Trigger speakerM{ [this] {return opertr.GetRightBumper();} };
	frc2::Trigger intakeMIgnoreSensor{ [this] {return opertr.GetAButton();} };


	frc2::Trigger increaseUpperAngleOffset{ [this] {return opertr.GetPOV() == 0;} };
	frc2::Trigger decreaseUpperAngleOffset{ [this] {return opertr.GetPOV() == 180;} };
	frc2::Trigger resetUpperAngleOffset{ [this] {return opertr.GetPOV() == 270;} };

	frc2::Trigger manualFrontalClimb{ [this] {return opertr.GetPOV() == 90;} };

	frc2::Trigger intakeM{ [this] {return opertr.GetRightTriggerAxis() > 0.1;} };

	// LED Triggers
	frc2::Trigger noteOnStorage{ [this] {return storage.isNoteOnForwardSensor();} };
	// frc2::Trigger shooterEmergencyMode{ [this] {return shooter.isEmergencyDisabled();} };
	frc2::Trigger storageSensorEmergencyMode{ [this] {return !storage.isSensorAvailable();} };
	frc2::Trigger intakeMotorActive{ [this] {return intake.getVoltage() != 0.0 && !storage.isNoteOnForwardSensor();} };

	//Autonomous
	frc2::CommandPtr defaultNoneAuto = frc2::cmd::None();
	frc2::CommandPtr center4NoteAuto = frc2::cmd::None();
	frc2::CommandPtr center5NoteAuto = frc2::cmd::None();
	frc2::CommandPtr center7NoteAuto = frc2::cmd::None();
	frc2::CommandPtr ampAuto = frc2::cmd::None();
	frc2::CommandPtr sourceAuto = frc2::cmd::None();

	frc2::CommandPtr ampAutoCenterRace = frc2::cmd::None();
	frc2::CommandPtr sourceAutoCenterRace = frc2::cmd::None();

	frc2::CommandPtr testingAuto = frc2::cmd::None();

	//Auto Chooser
	frc::SendableChooser<frc2::Command*> autoChooser;
};
