// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include <OvertureLib/Subsystems/Swerve/SwerveCharacterization/SwerveCharacterization.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include <OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h>

#include "Constants.h"

// class Chassis : public SwerveCharacterization {
class Chassis : public SwerveChassis {
public:
	Chassis();

private:
#ifndef __FRC_ROBORIO__
	SwerveModule backRight{ 5, 6, 11, -90, "BackRightModule", "OverCANivore" };
	SwerveModule backLeft{ 7, 8, 12, -90, "BackLeftModule", "OverCANivore" };
	SwerveModule frontLeft{ 1, 2, 9, -90, "FrontLeftModule", "OverCANivore" };
	SwerveModule frontRight{ 3, 4, 10, -90, "FrontRightModule", "OverCANivore" };
#else
	SwerveModule backRight{ 1, 2, 9, 28.388671875, "BackRightModule", "OverCANivore" };
	SwerveModule backLeft{ 3, 4, 10, 17.40234375, "BackLeftModule", "OverCANivore" };
	SwerveModule frontLeft{ 5, 6, 11, -158.818359375, "FrontLeftModule", "OverCANivore" };
	SwerveModule frontRight{ 7, 8, 12, -48.33984375, "FrontRightModule", "OverCANivore" };
#endif


	std::array<frc::Translation2d, 4> modulePos{
	 ChassisConstants::FrontLeftModuleTranslation,   //Front Left
	 ChassisConstants::FrontRightModuleTranslation,   //Front Right
	 ChassisConstants::BackRightModuleTranslation,   //Back Right
	 ChassisConstants::BackLeftModuleTranslation,   //Back Left
	};

	OverPigeon chassisPigeon{ 13, "OverCANivore" };
};
