// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructure.h"
#include <frc/MathUtil.h>
#include <thread>

SuperStructure::SuperStructure() {
	// Configure Motors
	m_lowerRight.setSupplyCurrentLimit(true, 20, 30, 0.5);
	m_lowerRight.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);

	m_lowerLeft.setSupplyCurrentLimit(true, 20, 30, 0.5);
	m_lowerLeft.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);
	m_lowerLeft.setFollow(m_lowerRight.GetDeviceID(), true);

	m_upperMotor.setSupplyCurrentLimit(true, 20, 30, 0.5);
	m_upperMotor.setSensorToMechanism(UPPER_GEAR_BOX_REDUCTION);

	// COnfigure Motion Magic and PID
	m_lowerRight.setPIDValues(80.0, 0.0, 0.0, 0.0, 0.0);
	m_lowerRight.configureMotionMagic(30.0, 10.0, 0.0);

	m_upperMotor.setPIDValues(30.0, 0.0, 0.0, 0.0, 0.0);
	m_upperMotor.configureMotionMagic(30.0, 10.0, 0.0);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	m_lowerRight.setSensorPosition(convertAngleToFalconPos(getLowerAngle()));
	std::this_thread::sleep_for(std::chrono::seconds(2));
	m_upperMotor.setSensorPosition(convertAngleToFalconPos(getUpperAngle()));

	setTargetCoord({ getLowerAngle(), getUpperAngle() });
}

void SuperStructure::setTargetCoord(SuperStructureState targetCoord) {
	m_TargetState = targetCoord;
}

void SuperStructure::setLowerAngleConstraints(double velocity, double acceleration){
	m_lowerRight.configureMotionMagic(velocity, acceleration, 0.0);
}

void SuperStructure::setUpperAngleConstraints(double velocity, double acceleration){
	m_upperMotor.configureMotionMagic(velocity, acceleration, 0.0);
}

double SuperStructure::getLowerAngle() {
	return (lowerEncoder.GetAbsolutePosition() - lowerOffset) * 360;
}

double SuperStructure::getUpperAngle() {
	return (upperEncoder.GetAbsolutePosition() - upperOffset) * 360;
}

SuperStructurePosition SuperStructure::getPosition() {
	return position;
}

void SuperStructure::setPosition(SuperStructurePosition pos) {
	position = pos;
}

SuperStructureState SuperStructure::getCurrentState() {
	SuperStructureState state;
	state.lowerAngle = getLowerAngle();
	state.upperAngle = getUpperAngle();
	return state;
}

void SuperStructure::setFalconTargetPos(SuperStructureState targetState) {
	m_lowerRight.setMotionMagicPosition(convertAngleToFalconPos(targetState.lowerAngle), lowerFF * cos(targetState.lowerAngle), false);
	m_upperMotor.setMotionMagicPosition(convertAngleToFalconPos(targetState.upperAngle), upperFF * cos(upperAngleFFCalculation(targetState.upperAngle)), false);
}

double SuperStructure::convertAngleToFalconPos(double angle) {
	return angle / 360;
}

double SuperStructure::upperAngleFFCalculation(double angle) {
	double rawVal = angle;
	rawVal = frc::InputModulus(rawVal, -180.0, 180.0);
	double angleToLower = rawVal;

	return angleToLower + getLowerAngle();
}

// This method will be called once per scheduler run
void SuperStructure::Periodic() {


	setFalconTargetPos(m_TargetState);

	frc::SmartDashboard::PutNumber("SuperStructure/Target/Lower Angle", m_TargetState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/Target/Upper Angle", m_TargetState.upperAngle);

	// Debugging
	SuperStructureState currentState = getCurrentState();
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Lower Angle", currentState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Upper Angle", currentState.upperAngle);
}
