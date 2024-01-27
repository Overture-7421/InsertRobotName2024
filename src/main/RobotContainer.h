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
#include "Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "Commands/ResetAngle/ResetAngle.h"
#include "Commands/MoveStorage/MoveStorage.h"
#include "Commands/ShootShooter/ShootShooter.h"
#include "Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "Commands/Climbing/Climbing.h"

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


	frc2::Trigger superStructureTest{ [this] {return driver.GetAButton();} };


	// Mechanism Commands
	frc2::Trigger intakePosition{ [this] {return opertr.GetYButton();} }; // Change to right trigger
	frc2::Trigger shootingPose{ [this] {return opertr.GetXButton();} };
	frc2::Trigger moveStorage{ [this] {return opertr.GetAButton();} };
	frc2::Trigger moveStorageInverted{ [this] {return opertr.GetBButton();} };
	frc2::Trigger shootshooter{ [this] {return opertr.GetRightBumper();} };
	frc2::Trigger shooterAngle{ [this] {return opertr.GetLeftBumper();} };

	//Auto Chooser
	frc::SendableChooser<std::string> autoChooser;

	double angleShooting;
};
