// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArms.h"
#include <frc/MathUtil.h>
#include <thread>

SupportArms::SupportArms() {
	// Configure Motors
	m_lowerRight.setSupplyCurrentLimit(true, 20, 30, 0.5);	
	m_lowerRight.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);

	m_lowerLeft.setSupplyCurrentLimit(true, 20, 30, 0.5);
	m_lowerLeft.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);
	m_lowerLeft.setFollow(m_lowerRight.GetDeviceID(), true);

	// COnfigure Motion Magic and PID
	m_lowerRight.setPIDValues(0.0, 0.0, 0.0, 0.0, 0.0);
	m_lowerRight.configureMotionMagic(0, 0.0, 0.0);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	m_lowerRight.setSensorPosition(getLowerAngle());

	setTargetCoord({ getLowerAngle() });
}

void SupportArms::setTargetCoord(SupportArmsState targetCoord) {
	m_TargetState = targetCoord;
}

double SupportArms::getLowerAngle() {
	return (lowerEncoder.GetAbsolutePosition() - lowerOffset) * 360;
}

SupportArmsPosition SupportArms::getPosition() {
	return position;
}

void SupportArms::setPosition(SupportArmsPosition pos) {
	position = pos;
}

SupportArmsState SupportArms::getCurrentState() {
	SupportArmsState state;
	state.lowerAngle = getLowerAngle();
	return state;
}

void SupportArms::setFalconTargetPos(SupportArmsState targetState) {
	m_lowerRight.setMotionMagicPosition(convertAngleToFalconPos(targetState.lowerAngle), lowerFF * cos(targetState.lowerAngle), false);
}

double SupportArms::convertAngleToFalconPos(double angle) {
	return angle / 360;
}

// This method will be called once per scheduler run
void SupportArms::Periodic() {
	setFalconTargetPos(m_TargetState);

	// frc::SmartDashboard::PutNumber("Target /Lower Angle", m_TargetState.lowerAngle);

	// // Debugging
	// SupportArmsState currentState = getCurrentState();
	// frc::SmartDashboard::PutNumber("Current /Lower Angle", currentState.lowerAngle);

	// frc::SmartDashboard::PutNumber("Position", position);
}
