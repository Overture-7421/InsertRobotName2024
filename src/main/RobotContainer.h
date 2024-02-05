// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Vision/AprilTagCamera.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/SupportArms/SupportArms.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Commands/ResetAngle/ResetAngle.h"

#include "Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"

#include "Commands/GroundGrabCommand/GroundGrabCommand.h"
#include "Commands/SourceGrabCommand/SourceGrabCommand.h"
#include "Commands/AmpCommand/AmpCommand.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/SpeakerCommand/SpeakerCommand.h"
#include "Commands/VisionAmpCommand/VisionAmpCommand.h"
#include "Commands/VisionSourceGrabCommand/VisionSourceGrabCommand.h"

#include "Commands/Climbing/Climbing.h"
#include "Commands/TrapShoot/TrapShoot.h"

#include "OvertureLib/Commands/Drive/Drive.h"

class RobotContainer {
public:
	RobotContainer();

	frc2::CommandPtr GetAutonomousCommand();
private:
	void ConfigureBindings();

	// Subsystems
	Chassis chassis;
	AprilTagCamera aprilTagCamera{ &chassis };
	Intake intake;
	SuperStructure superStructure;
	SupportArms supportArms;
	Storage storage;
	Shooter shooter;

	// Controllers
	frc::XboxController driver{ 0 };
	frc::XboxController opertr{ 1 };


	// Driver Commands
	frc2::Trigger resetAngleButton{ [this] {return driver.GetBackButton();} };
	frc2::Trigger climbButton{ [this] {return driver.GetStartButton();} };
	frc2::Trigger shootTrap{ [this] {return driver.GetLeftBumper();} };


	// Mechanism Commands

	frc2::Trigger groundGrab{ [this] {return opertr.GetLeftTriggerAxis() > 0.3;} };
	frc2::Trigger sourceGrab{ [this] {return opertr.GetRightTriggerAxis() > 0.3;} };
	frc2::Trigger ampShoot{ [this] {return opertr.GetLeftBumper();} };
	frc2::Trigger speakerShoot{ [this] {return opertr.GetRightBumper();} };
	frc2::Trigger storageIn{ [this] {return opertr.GetYButton();} };
	frc2::Trigger ampButton{ [this] {return opertr.GetBackButton();} };
	frc2::Trigger sourceButton{ [this] {return opertr.GetXButton();} };
	frc2::Trigger manualClimb{ [this] {return opertr.GetBButton();} };


	//Auto Chooser
	frc::SendableChooser<std::string> autoChooser;

	double angleShooting;
};
