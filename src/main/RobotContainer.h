// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <frc/XboxController.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Vision/AprilTagCamera.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/SupportArms/SupportArms.h"
#include "Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "Commands/ResetAngle/ResetAngle.h"
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

	// Controllers
	frc::XboxController driver{ 0 };

	// Driver Commands
	frc2::Trigger resetAngleButton{ [this] {return driver.GetBackButton();} };

	// Testing
	frc2::Trigger stopintake{ [this] {return driver.GetBButton();} };
	frc2::Trigger startintake{ [this] {return driver.GetXButton();} };
	frc2::Trigger movestructure{ [this] {return driver.GetYButton();} };
	frc2::Trigger movearms{ [this] {return driver.GetLeftBumper();} };


	//Auto Chooser
	frc::SendableChooser<std::string> autoChooser;
};
