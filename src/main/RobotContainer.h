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
#include "Commands/VisionSpeakerCommand/VisionSpeakerCommand.h"

#include "Commands/Climbing/Climbing.h"
#include "Commands/TrapShoot/TrapShoot.h"

#include "OvertureLib/Commands/Drive/Drive.h"
#include "OvertureLib/Characterization/SysIDRoutineBot.h"

class RobotContainer : public SysIDRoutineBot {
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
	frc2::Trigger ampV{ [this] {return driver.GetLeftTriggerAxis() > 0.3;} };
	frc2::Trigger sourceV{ [this] {return driver.GetRightBumper();} };
	frc2::Trigger speakerV{ [this] {return driver.GetRightTriggerAxis() > 0.3;} };	// TO GET TESTED

	// Mechanism Commands
	frc2::Trigger ampM{ [this] {return opertr.GetLeftBumper();} };
	frc2::Trigger sourceM{ [this] {return opertr.GetBButton();} };
	frc2::Trigger climbV{ [this] {return opertr.GetXButton();} };
	frc2::Trigger climbM{ [this] {return opertr.GetYButton();} };
	frc2::Trigger shootM{ [this] {return opertr.GetLeftTriggerAxis() > 0.3;} };
	frc2::Trigger speakerM{ [this] {return opertr.GetRightBumper();} };
	frc2::Trigger trapV{ [this] {return opertr.GetAButton();} };
	frc2::Trigger closed{ [this] {return opertr.GetPOV() == 0;} };
	frc2::Trigger intakeM{ [this] {return opertr.GetRightTriggerAxis() > 0.3;} };

	//Auto Chooser
	frc::SendableChooser<std::string> autoChooser;

	double angleShooting;
};
