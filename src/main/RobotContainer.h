// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <frc/XboxController.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "Subsystems/Chassis/Chassis.h"

#include "OvertureLib/Commands/Drive/Drive.h"

class RobotContainer {
public:
	RobotContainer();

	frc2::CommandPtr GetAutonomousCommand();

private:
	void ConfigureBindings();

	// Subsystems
	Chassis chassis;

	// Controllers
	frc::XboxController driver{ 0 };

	// Driver Commands
	frc2::Trigger resetAngleButton{ [this] {return driver.GetBackButton();} };

	//Auto Chooser
	frc::SendableChooser<std::string> autoChooser;
};
