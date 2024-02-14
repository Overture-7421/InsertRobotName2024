// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructure.h"
#include <frc/MathUtil.h>
#include <thread>
#include <iostream>

#define DEG_TO_RAD M_PI / 180.0

SuperStructure::SuperStructure() {
	// Configure Motors
	lowerLeftMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	lowerLeftMotor.setSensorToMechanism(SuperStructureConstants::LowerAngleGearRatio);

	lowerRightMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	lowerRightMotor.setSensorToMechanism(SuperStructureConstants::LowerAngleGearRatio);

	lowerRightMotor.setFollow(lowerLeftMotor.GetDeviceID(), true);

	upperMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	upperMotor.setSensorToMechanism(SuperStructureConstants::UpperAngleGearRatio);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	lowerLeftMotor.setSensorPosition(convertAngleToFalconPos(getLowerAngle()));
	std::this_thread::sleep_for(std::chrono::seconds(2));
	upperMotor.setSensorPosition(convertAngleToFalconPos(getUpperAngle()));

	SoftwareLimitSwitchConfigs lowerMotorSoftLimitConfig;
	lowerMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ForwardSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::LowerAngleUpperLimit);

	lowerMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ReverseSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::LowerAngleLowerLimit);

	lowerLeftMotor.configureSoftwareLimitSwitch(lowerMotorSoftLimitConfig);
	
	SoftwareLimitSwitchConfigs upperMotorSoftLimitConfig;
	upperMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ForwardSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::UpperAngleUpperLimit);

	upperMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ReverseSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::UpperAngleLowerLimit);

	upperMotor.configureSoftwareLimitSwitch(upperMotorSoftLimitConfig);

	lowerLeftMotor.setContinuousWrap();
	upperMotor.setContinuousWrap();

	setTargetCoord({ getLowerAngle(), getUpperAngle() });

	// Configure Motion Magic and PID
	lowerLeftMotor.setPIDValues(390.0, 0.0, 0.0, 0.0, 0.0);
	lowerLeftMotor.configureMotionMagic(1.0, 1.0, 0.0);

	upperMotor.setPIDValues(250.0, 0.0, 0.0, 0.0, 0.0);
	upperMotor.configureMotionMagic(1.0, 1.0, 0.0);
}

void SuperStructure::setTargetCoord(SuperStructureState targetState) {
	this->targetState = targetState;
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

SuperStructureState SuperStructure::getCurrentState() {
	SuperStructureState state;
	state.lowerAngle = getLowerAngle();
	state.upperAngle = getUpperAngle();
	return state;
}

void SuperStructure::setFalconTargetPos(SuperStructureState targetState, SuperStructureState currentState) {
	lowerLeftMotor.setMotionMagicPosition(convertAngleToFalconPos(targetState.lowerAngle), lowerFF.Calculate(units::degree_t(targetState.lowerAngle), units::radians_per_second_t(0)).value(), false);
	upperMotor.setMotionMagicPosition(convertAngleToFalconPos(targetState.upperAngle), upperFF.Calculate(units::degree_t(targetState.upperAngle) + upperFFOffset, units::radians_per_second_t(0)).value(), false);
}

double SuperStructure::convertAngleToFalconPos(double angle) {
	return angle / 360.0;
}

// This method will be called once per scheduler run
void SuperStructure::Periodic() {
	SuperStructureState currentState = getCurrentState();
	SuperStructureState actualTarget = targetState;

	actualTarget.lowerAngle = std::clamp(actualTarget.lowerAngle, SuperStructureConstants::LowerAngleLowerLimit, SuperStructureConstants::LowerAngleUpperLimit);
	actualTarget.upperAngle = std::clamp(actualTarget.upperAngle, SuperStructureConstants::UpperAngleLowerLimit, SuperStructureConstants::UpperAngleUpperLimit);

	if (currentState.lowerAngle < SuperStructureConstants::LowerAngleSafetyThreshold && actualTarget.upperAngle < SuperStructureConstants::UpperAngleSafetyLimit){
		actualTarget.upperAngle = SuperStructureConstants::UpperAngleSafetyLimit;
	}

	frc::SmartDashboard::PutNumber("SuperStructure/Current/Lower", currentState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Upper", currentState.upperAngle);

	frc::SmartDashboard::PutNumber("SuperStructure/Current/LowerMotor", lowerLeftMotor.GetPosition().GetValueAsDouble());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/UpperMotor", upperMotor.GetPosition().GetValueAsDouble());

	frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Lower", targetState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Upper", targetState.upperAngle);

	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Lower", actualTarget.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Upper", actualTarget.upperAngle);

	setFalconTargetPos(actualTarget, currentState);
}
