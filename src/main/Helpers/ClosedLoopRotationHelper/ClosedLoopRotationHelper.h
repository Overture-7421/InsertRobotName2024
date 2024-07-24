// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Robots/OverRobot/RobotConstants.h>
#include <units/angular_acceleration.h>

class ClosedLoopRotationHelper : public SpeedsHelper {
public:
	ClosedLoopRotationHelper();
	void setCurrentAngle(units::degree_t position);
	void alterSpeed(frc::ChassisSpeeds& inputSpeed) override;

private:

	frc::ProfiledPIDController<units::degree> controller{ 0.1, 0.0, 0.0, {20_deg_per_s, 40_deg_per_s_sq}, RobotConstants::LoopTime };
	units::degree_t m_currentAngle;
};
