// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructure.h"
#include <frc/MathUtil.h>
#include <thread>

#define DEG_TO_RAD M_PI / 180.0

SuperStructure::SuperStructure() {
	// Configure Motors
	m_lowerRight.setSupplyCurrentLimit(true, 30, 40, 3);
	m_lowerRight.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);

	m_lowerLeft.setSupplyCurrentLimit(true, 30, 40, 3);
	m_lowerLeft.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);

	m_lowerRight.setFollow(m_lowerLeft.GetDeviceID(), true);

	m_upperMotor.setSupplyCurrentLimit(true, 30, 40, 3);
	m_upperMotor.setSensorToMechanism(UPPER_GEAR_BOX_REDUCTION);


	std::this_thread::sleep_for(std::chrono::seconds(2));
	m_lowerLeft.setSensorPosition(convertAngleToFalconPos(getLowerAngle()));
	std::this_thread::sleep_for(std::chrono::seconds(2));
	m_upperMotor.setSensorPosition(convertAngleToFalconPos(getUpperAngle()));

	SoftwareLimitSwitchConfigs upperMotorSoftLimitConfig;
	upperMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ForwardSoftLimitThreshold = 0;

	upperMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ReverseSoftLimitThreshold = -0.548096;

	m_upperMotor.configureSoftwareLimitSwitch(upperMotorSoftLimitConfig);

	SoftwareLimitSwitchConfigs lowerMotorSoftLimitConfig;
	lowerMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ForwardSoftLimitThreshold = 0.256592;

	lowerMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ReverseSoftLimitThreshold = -0.090088;

	m_lowerLeft.configureSoftwareLimitSwitch(lowerMotorSoftLimitConfig);

	m_lowerLeft.setContinuousWrap();
	m_upperMotor.setContinuousWrap();

	setTargetCoord({ getLowerAngle(), getUpperAngle() });

	// Configure Motion Magic and PID
	m_lowerLeft.setPIDValues(350, 0.0, 0.0, 0.0, 0.0);
	m_lowerLeft.configureMotionMagic(15.0, 40.0, 0.0);

	m_upperMotor.setPIDValues(210.0, 0.0, 0.0, 0.0, 0.0);
	m_upperMotor.configureMotionMagic(15.0, 40.0, 0.0);
}

void SuperStructure::setTargetCoord(SuperStructureState targetCoord) {
	m_TargetState = targetCoord;
}

double SuperStructure::getLowerAngle() {
	double rawLowerEncoder = lowerEncoder.GetAbsolutePosition() + lowerOffset; // Goes from 0 to 1
	double degrees = rawLowerEncoder * 360.0;
	return frc::InputModulus(degrees, -180.0, 180.0);
}

double SuperStructure::getUpperAngle() {
	double rawUpperEncoder = upperEncoder.GetAbsolutePosition() + upperOffset; // Goes from 0 to 1
	double degrees = rawUpperEncoder * 360.0;
	return -frc::InputModulus(degrees, -180.0, 180.0);
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

void SuperStructure::setFalconTargetPos(SuperStructureState targetState, SuperStructureState currentState) {
	m_lowerLeft.setMotionMagicPosition(convertAngleToFalconPos(targetState.lowerAngle), lowerFF * cos(currentState.lowerAngle * DEG_TO_RAD), true);
 	m_upperMotor.setMotionMagicPosition(convertAngleToFalconPos(targetState.upperAngle), upperFF * cos((currentState.lowerAngle + currentState.upperAngle + 90.0) * DEG_TO_RAD), true);

	frc::SmartDashboard::PutNumber("SuperStructure/FalconTarget/Lower", convertAngleToFalconPos(targetState.lowerAngle));
	frc::SmartDashboard::PutNumber("SuperStructure/FalconTarget/Upper", convertAngleToFalconPos(targetState.upperAngle));
}

double SuperStructure::convertAngleToFalconPos(double angle) {
	return angle / 360.0;
}

// This method will be called once per scheduler run
void SuperStructure::Periodic() {

	SuperStructureState currentState = getCurrentState();

	SuperStructureState actualTarget = m_TargetState;

	if (m_TargetState.lowerAngle < -29) {
		actualTarget.lowerAngle = -29;
	} else if (m_TargetState.lowerAngle > 90) {
		actualTarget.lowerAngle = 90;
	}

	if (m_TargetState.upperAngle < -110) {
		actualTarget.upperAngle = -110;
	} else if (m_TargetState.upperAngle > 0) {
		actualTarget.upperAngle = 0;
	}

	if (currentState.lowerAngle < -23 && actualTarget.upperAngle < -15){
		actualTarget.upperAngle = -15;
	}

	// setFalconTargetPos(actualTarget, currentState);

	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Lower", actualTarget.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Upper", actualTarget.upperAngle);

	frc::SmartDashboard::PutNumber("SuperStructure/Target/Lower", m_TargetState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/Target/Upper", m_TargetState.upperAngle);

	// Debugging
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Lower", currentState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Upper", currentState.upperAngle);

	frc::SmartDashboard::PutNumber("SuperStructure/Current/Lower Motor Position", m_lowerLeft.GetPosition().GetValueAsDouble());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Upper Motor Position", m_upperMotor.GetPosition().GetValueAsDouble());

}
