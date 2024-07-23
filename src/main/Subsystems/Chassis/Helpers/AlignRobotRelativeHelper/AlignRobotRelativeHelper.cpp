// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignRobotRelativeHelper.h"

AlignRobotRelativeHelper::AlignRobotRelativeHelper() {};

void AlignRobotRelativeHelper::setCurrentAngle(units::degree_t position) {
	m_currentAngle = position;
}

void AlignRobotRelativeHelper::alterSpeed(frc::ChassisSpeeds& inputSpeed) {
	double out = controller.Calculate(units::degree_t(m_currentAngle), units::degree_t(0));

	if (controller.AtSetpoint()) {
		out = 0;
	}

	inputSpeed.vy += units::meters_per_second_t(out);
}
