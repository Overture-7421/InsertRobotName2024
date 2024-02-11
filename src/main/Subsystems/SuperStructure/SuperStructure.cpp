// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructure.h"
#include <frc/MathUtil.h>
#include <thread>

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
	lowerRightMotor.setSensorPosition(convertAngleToFalconPos(getLowerAngle()));
	std::this_thread::sleep_for(std::chrono::seconds(2));
	upperMotor.setSensorPosition(convertAngleToFalconPos(getUpperAngle()));

	SoftwareLimitSwitchConfigs upperMotorSoftLimitConfig;
	upperMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ForwardSoftLimitThreshold = 0;

	upperMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ReverseSoftLimitThreshold = -0.548096;

	upperMotor.configureSoftwareLimitSwitch(upperMotorSoftLimitConfig);

	SoftwareLimitSwitchConfigs lowerMotorSoftLimitConfig;
	lowerMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ForwardSoftLimitThreshold = 0.256592;

	lowerMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ReverseSoftLimitThreshold = -0.090088;

	lowerLeftMotor.configureSoftwareLimitSwitch(lowerMotorSoftLimitConfig);

	lowerLeftMotor.setContinuousWrap();
	upperMotor.setContinuousWrap();

	setTargetCoord({ getLowerAngle(), getUpperAngle() });

	// Configure Motion Magic and PID
	lowerLeftMotor.setPIDValues(350, 0.0, 0.0, 0.0, 0.0);
	lowerLeftMotor.configureMotionMagic(15.0, 40.0, 0.0);

	upperMotor.setPIDValues(210.0, 0.0, 0.0, 0.0, 0.0);
	upperMotor.configureMotionMagic(15.0, 40.0, 0.0);
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
	lowerLeftMotor.setMotionMagicPosition(convertAngleToFalconPos(targetState.lowerAngle), lowerFF * std::cos(currentState.lowerAngle * DEG_TO_RAD), false);
	upperMotor.setMotionMagicPosition(convertAngleToFalconPos(targetState.upperAngle), upperFF * std::cos((currentState.lowerAngle + currentState.upperAngle + 90.0) * DEG_TO_RAD), false);
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

	setFalconTargetPos(actualTarget, currentState);
}
