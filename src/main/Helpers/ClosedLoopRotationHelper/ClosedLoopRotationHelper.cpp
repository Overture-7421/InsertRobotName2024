// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedLoopRotationHelper.h"

ClosedLoopRotationHelper::ClosedLoopRotationHelper() {
	controller.EnableContinuousInput(-180_deg, 180_deg);
	controller.SetTolerance(2_deg, 2_deg);
};

void ClosedLoopRotationHelper::setCurrentAngle(units::degree_t position) {
	m_currentAngle = position;
}

void ClosedLoopRotationHelper::alterSpeed(frc::ChassisSpeeds& inputSpeed) {
	double out = controller.Calculate(units::degree_t(m_currentAngle), units::degree_t(0));

	if (controller.AtGoal()) {
		out = 0;
	}

	inputSpeed.omega = units::radians_per_second_t(out);
}
