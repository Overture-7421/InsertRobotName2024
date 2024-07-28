// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedLoopRotationHelper.h"

ClosedLoopRotationHelper::ClosedLoopRotationHelper() {
	controller.EnableContinuousInput(-180_deg, 180_deg);
	controller.SetTolerance(2_deg);
	controller.SetIZone(3);
};

void ClosedLoopRotationHelper::setTargetAngle(units::radian_t goal, units::radian_t current) {
	m_TargetAngle = goal;
	m_CurrentAnlge = current;

}

void ClosedLoopRotationHelper::alterSpeed(frc::ChassisSpeeds& inputSpeed) {
	double out = controller.Calculate(m_CurrentAnlge, m_TargetAngle);

	if (controller.AtGoal()) {
		out = 0;
	}

	inputSpeed.omega = units::radians_per_second_t(out);
}
