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
	lowerLeftMotor.setSensorPosition(lowerCANCoder.getSensorAbsolutePosition());
	upperMotor.setSensorPosition(upperCANCoder.getSensorAbsolutePosition());

	lowerLeftMotor.setFusedCANCoder(lowerCANCoder.GetDeviceID());
	lowerLeftMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	lowerLeftMotor.setRotorToSensorRatio(SuperStructureConstants::LowerAngleGearRatio);

	lowerRightMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	lowerRightMotor.setSensorToMechanism(SuperStructureConstants::LowerAngleGearRatio);

	lowerRightMotor.setFollow(lowerLeftMotor.GetDeviceID(), true);

	upperMotor.setFusedCANCoder(upperCANCoder.GetDeviceID());
	upperMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	upperMotor.setRotorToSensorRatio(SuperStructureConstants::UpperAngleGearRatio);

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

	setTargetCoord({ lowerCANCoder.getSensorAbsolutePosition() * 360.0, upperCANCoder.getSensorAbsolutePosition() * 360.0 });

	// Configure Motion Magic and PID
	lowerLeftMotor.setPIDValues(200.0, 70.0, 0.0, 0.0, 0.0);
	lowerLeftMotor.configureMotionMagic(1.0, 6.0, 0.0);

	upperMotor.setPIDValues(220.0, 70.0, 0.0, 0.0, 0.0);
	upperMotor.configureMotionMagic(1.0, 6.0, 0.0);
}

void SuperStructure::setTargetCoord(SuperStructureState targetState) {
	this->targetState = targetState;
}

// double SuperStructure::getLowerAngle() {
// 	double rawLowerEncoder = lowerLeftMotor.GetPosition().GetValueAsDouble(); // Goes from 0 to 1
// 	double degrees = rawLowerEncoder * 360.0;
// 	return frc::InputModulus(degrees, -180.0, 180.0);
// }

// double SuperStructure::getUpperAngle() {
// 	double rawUpperEncoder = upperMotor.GetPosition().GetValueAsDouble(); // Goes from 0 to 1
// 	double degrees = rawUpperEncoder * 360.0;
// 	return frc::InputModulus(degrees, -180.0, 180.0);
// }

double SuperStructure::getLowerAngle() {
	double rawLowerEncoder = lowerLeftMotor.getPosition(); // Goes from 0 to 1
	double degrees = rawLowerEncoder * 360.0;
	return frc::InputModulus(degrees, -180.0, 180.0);
}

double SuperStructure::getUpperAngle() {
	double rawUpperEncoder = upperMotor.getPosition(); // Goes from 0 to 1
	double degrees = rawUpperEncoder * 360.0;
	return frc::InputModulus(degrees, -180.0, 180.0);
}

SuperStructureState SuperStructure::getCurrentState() {
	SuperStructureState state;
	state.lowerAngle = getLowerAngle();
	state.upperAngle = getUpperAngle();
	return state;
}

double SuperStructure::convertAngleToFalconPos(double angle) {
	return angle / 360.0;
}

// This method will be called once per scheduler run
void SuperStructure::Periodic() {
	currentState = getCurrentState();
	actualTarget = targetState;

	actualTarget.lowerAngle = std::clamp(actualTarget.lowerAngle, SuperStructureConstants::LowerAngleLowerLimit, SuperStructureConstants::LowerAngleUpperLimit);
	actualTarget.upperAngle = std::clamp(actualTarget.upperAngle, SuperStructureConstants::UpperAngleLowerLimit, SuperStructureConstants::UpperAngleUpperLimit);

	if (currentState.lowerAngle < SuperStructureConstants::LowerAngleSafetyThreshold && actualTarget.upperAngle < SuperStructureConstants::UpperAngleSafetyLimit) {
		actualTarget.upperAngle = SuperStructureConstants::UpperAngleSafetyLimit;
	}

	lowerLeftMotor.setMotionMagicPosition(convertAngleToFalconPos(actualTarget.lowerAngle), 0, true);
	upperMotor.setMotionMagicPosition(convertAngleToFalconPos(actualTarget.upperAngle), 0, true);
}

void SuperStructure::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Lower", currentState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Upper", currentState.upperAngle);

	//frc::SmartDashboard::PutNumber("SuperStructure/Current/RawLower", lowerCANCoder.getSensorAbsolutePosition());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/LowerMotorPosition", lowerLeftMotor.GetPosition().GetValue().value());
	//frc::SmartDashboard::PutNumber("SuperStructure/Current/RawUpper", upperCANCoder.getSensorAbsolutePosition());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/UpperMotorPosition", upperMotor.GetPosition().GetValue().value());


	//frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Lower", targetState.lowerAngle);
	//frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Upper", targetState.upperAngle);

	//frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Lower", actualTarget.lowerAngle);
	//frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/LowerMotor", convertAngleToFalconPos(actualTarget.lowerAngle));

	//frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Upper", actualTarget.upperAngle);
	//frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/UpperMotor", convertAngleToFalconPos(actualTarget.upperAngle));

}