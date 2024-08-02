// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignRobotRelativeHelper.h"

AlignRobotRelativeHelper::AlignRobotRelativeHelper() {
	controller.SetTolerance(1_deg);
};

void AlignRobotRelativeHelper::setCurrentAngle(units::degree_t position) {
	m_currentAngle = position;
}

void AlignRobotRelativeHelper::alterSpeed(frc::ChassisSpeeds& inputSpeed) {
	if(!noteDetected) {
		return;
	}

	double out = controller.Calculate(m_currentAngle, units::degree_t(0));

	if (controller.AtGoal()) {
		out = 0;
	}

	inputSpeed.vy += units::meters_per_second_t(out);
}

bool AlignRobotRelativeHelper::isNoteDetected() {
	return noteDetected;
}

void AlignRobotRelativeHelper::setNoteDetected(units::degree_t initialPosition) {
	noteDetected = true;
	controller.Reset(initialPosition);
	m_currentAngle = initialPosition;
}

void AlignRobotRelativeHelper::setNoteLost() {
	noteDetected = false;
}