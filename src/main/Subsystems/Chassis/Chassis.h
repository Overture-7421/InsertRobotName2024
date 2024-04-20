// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include <Subsystems/Swerve/SwerveCharacterization/SwerveCharacterization.h>
#include <Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include <Subsystems/Swerve/SwerveModule/SwerveModule.h>

#include "Constants.h"

// class Chassis : public SwerveCharacterization {
class Chassis : public SwerveChassis {
public:
	Chassis();

private:
#ifndef __FRC_ROBORIO__
	SwerveModule backRight{ 5, 6, 11, -90_deg, "BackRightModule", "OverCANivore" };
	SwerveModule backLeft{ 7, 8, 12, -90_deg, "BackLeftModule", "OverCANivore" };
	SwerveModule frontLeft{ 1, 2, 9, -90_deg, "FrontLeftModule", "OverCANivore" };
	SwerveModule frontRight{ 3, 4, 10, -90_deg, "FrontRightModule", "OverCANivore" };
#else
	SwerveModule backRight{ 1, 2, 9, 	-152.578125_deg + 180_deg, "BackRightModule", "OverCANivore" };
	SwerveModule backLeft{ 3, 4, 10, 	-163.4765625_deg + 180_deg, "BackLeftModule", "OverCANivore" };
	SwerveModule frontLeft{ 5, 6, 11, 	-159.345703125_deg, "FrontLeftModule", "OverCANivore" };
	SwerveModule frontRight{ 7, 8, 12, 	133.2421875_deg + 180_deg, "FrontRightModule", "OverCANivore" };
#endif


	std::array<frc::Translation2d, 4> modulePos{
	 ChassisConstants::FrontLeftModuleTranslation,   //Front Left
	 ChassisConstants::FrontRightModuleTranslation,   //Front Right
	 ChassisConstants::BackRightModuleTranslation,   //Back Right
	 ChassisConstants::BackLeftModuleTranslation,   //Back Left
	};

	OverPigeon chassisPigeon{ 13, "OverCANivore" };
};
