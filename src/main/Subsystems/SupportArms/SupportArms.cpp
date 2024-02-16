// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArms.h"
#include <frc/MathUtil.h>
#include <thread>
#include <iostream>


#define DEG_TO_RAD M_PI / 180.0

SupportArms::SupportArms() {
	// Configure Motors
	lowerRightMotor.setSupplyCurrentLimit(true, 40, 60, 0.5);
	lowerRightMotor.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);

	// COnfigure Motion Magic and PID
	lowerRightMotor.setPIDValues(180, 0.0, 0.0, 0.0, 0.0);
	lowerRightMotor.configureMotionMagic(3.0, 3.0, 0.0);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	lowerRightMotor.setSensorPosition(convertAngleToFalconPos(getLowerAngle()));

	setTargetCoord({ getLowerAngle() });
}

void SupportArms::setTargetCoord(SupportArmsState targetCoord) {
	m_TargetState = targetCoord;
}

double SupportArms::getLowerAngle() {
	double rawLowerEncoder = lowerEncoder.GetAbsolutePosition() + lowerOffset; // Goes from 0 to 1
	double degrees = rawLowerEncoder * 360.0;
	return frc::InputModulus(degrees, -180.0, 180.0);
}

SupportArmsState SupportArms::getCurrentState() {
	SupportArmsState state;
	state.lowerAngle = getLowerAngle();
	return state;
}

void SupportArms::setFalconTargetPos(SupportArmsState targetState, SupportArmsState currentState) {
	lowerRightMotor.setMotionMagicPosition(convertAngleToFalconPos(targetState.lowerAngle), lowerFF.Calculate(units::degree_t(targetState.lowerAngle), units::radians_per_second_t(0)).value(), false);
}

double SupportArms::convertAngleToFalconPos(double angle) {
	return angle / 360.0;
}

// This method will be called once per scheduler run
void SupportArms::Periodic() {
	SupportArmsState currentState = getCurrentState();
	setFalconTargetPos(m_TargetState, currentState);

	// Debugging
	frc::SmartDashboard::PutNumber("SupportArms/Current/Angle", currentState.lowerAngle);
	frc::SmartDashboard::PutNumber("SupportArms/Current/Raw Encoder", lowerEncoder.GetAbsolutePosition());
	frc::SmartDashboard::PutNumber("SupportArms/Current/Motor Position", lowerEncoder.GetAbsolutePosition());
}
