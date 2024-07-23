// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignFieldRelativeHelper.h"

AlignFieldRelativeHelper::AlignFieldRelativeHelper(SwerveChassis* chassis)
	: m_chassis(chassis) {};

void AlignFieldRelativeHelper::setTargetPosition(units::meter_t xPosition, units::meter_t yPosition) {
	m_xPosition = xPosition;
	m_yPosition = yPosition;
}

void AlignFieldRelativeHelper::alterSpeed(frc::ChassisSpeeds& inputSpeed) {
	double xOut = xController.Calculate(m_chassis->getEstimatedPose().Translation().X(), m_xPosition);
	double yOut = yController.Calculate(m_chassis->getEstimatedPose().Translation().Y(), m_yPosition);

	xOut = std::clamp(xOut, -AllignToNoteConstants::maxOutput, AllignToNoteConstants::maxOutput);
	yOut = std::clamp(yOut, -AllignToNoteConstants::maxOutput, AllignToNoteConstants::maxOutput);

	if (xController.AtSetpoint()) {
		xOut = 0;
	}

	if (yController.AtSetpoint()) {
		yOut = 0;
	}

	inputSpeed.vx += units::meters_per_second_t(xOut);
	inputSpeed.vy += units::meters_per_second_t(yOut);
}
