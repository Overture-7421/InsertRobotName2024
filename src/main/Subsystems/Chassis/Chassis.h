// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include <OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h>

class Chassis : public SwerveChassis {
public:
	Chassis();

	void Periodic() override;

private:
	SwerveModule backRight{ 5, 6, 11, 17.841796875, "BackRightModule", "OverCANivore" };
	SwerveModule backLeft{ 7, 8, 12, -61.435546875, "BackLeftModule", "OverCANivore" };
	SwerveModule frontLeft{ 1, 2, 9, -73.564453125, "FrontLeftModule", "OverCANivore" };
	SwerveModule frontRight{ 3, 4, 10, -49.482421875, "FrontRightModule", "OverCANivore" };

	std::array<frc::Translation2d, 4> modulePos{
	 frc::Translation2d(10.39_in, 10.39_in),   //Front Left
		 frc::Translation2d(10.39_in, -10.39_in),   //Front Right
		 frc::Translation2d(-10.39_in, -10.39_in),   //Back Right
		 frc::Translation2d(-10.39_in, 10.39_in),   //Back Left
	};

	double turnRatio = 150 / 7;
	double driveRatio = 5.9027777;
	double wheelDiameter = 0.1016;

	Pigeon2 chassisPigeon{ 13, "OverCANivore" };
};
