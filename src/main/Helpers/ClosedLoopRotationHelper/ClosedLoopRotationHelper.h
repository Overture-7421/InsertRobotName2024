// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Robots/OverRobot/RobotConstants.h>
#include <units/angular_acceleration.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

class ClosedLoopRotationHelper : public SpeedsHelper {
public:
	ClosedLoopRotationHelper();
	void setTargetAngle(units::radian_t goal, units::radian_t current);
	void alterSpeed(frc::ChassisSpeeds& inputSpeed) override;

private:

	frc::ProfiledPIDController<units::radian> controller{ 7, 0, 0, {18_rad_per_s , 12_rad_per_s_sq }, RobotConstants::LoopTime };
	units::radian_t m_TargetAngle;
	units::radian_t m_CurrentAnlge;
};
