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
	lowerLeftMotor.setFusedCANCoder(lowerCANCoder.GetDeviceID());
	lowerLeftMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	lowerLeftMotor.setRotorToSensorRatio(SuperStructureConstants::LowerAngleGearRatio);

	lowerRightMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	lowerRightMotor.setSensorToMechanism(SuperStructureConstants::LowerAngleGearRatio);

	lowerRightMotor.setFollow(lowerLeftMotor.GetDeviceID(), true);

	upperMotor.setSupplyCurrentLimit(true, 30, 40, 0.5);
	upperMotor.setSensorToMechanism(SuperStructureConstants::UpperAngleGearRatio);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	lowerLeftMotor.setSensorPosition(convertAngleToFalconPos(getLowerAngle()));
	std::this_thread::sleep_for(std::chrono::seconds(2));
	upperMotor.setSensorPosition(convertAngleToFalconPos(getUpperAngle()));
	upperMotor.GetPosition().SetUpdateFrequency(100_Hz);

	SoftwareLimitSwitchConfigs lowerMotorSoftLimitConfig;
	lowerMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ForwardSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::LowerAngleUpperLimit);

	lowerMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ReverseSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::LowerAngleLowerLimit);

	lowerLeftMotor.configureSoftwareLimitSwitch(lowerMotorSoftLimitConfig);

	// SoftwareLimitSwitchConfigs upperMotorSoftLimitConfig;
	// upperMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	// upperMotorSoftLimitConfig.ForwardSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::UpperAngleUpperLimit);

	// upperMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	// upperMotorSoftLimitConfig.ReverseSoftLimitThreshold = convertAngleToFalconPos(SuperStructureConstants::UpperAngleLowerLimit);

	// upperMotor.configureSoftwareLimitSwitch(upperMotorSoftLimitConfig);

	setTargetCoord({ getLowerAngle(), getUpperAngle() });

	// Configure Motion Magic and PID
	lowerLeftMotor.setPIDValues(390.0, 0.0, 0.0, 0.0, 0.0);
	lowerLeftMotor.configureMotionMagic(1.0, 3.0, 0.0);


	// upperMotor.setPIDValues(270.0, 0.0, 0.0, 0.0, 0.0);
	// upperMotor.configureMotionMagic(1.0, 6.0, 0.0);

	frc::SmartDashboard::PutData("SuperStructure/UpperPID", &upperPID);

	upperPID.EnableContinuousInput(-180_deg, 180_deg);

	upperPID.SetIZone(3);
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
	double rawLowerEncoder = lowerCANCoder.getSensorAbsolutePosition(); // Goes from 0 to 1
	double degrees = rawLowerEncoder * 360.0;
	return frc::InputModulus(degrees, -180.0, 180.0);
}

double SuperStructure::getUpperAngle() {
	double rawUpperEncoder = upperEncoder.GetAbsolutePosition() + upperOffset; // Goes from 0 to 1
	double degrees = rawUpperEncoder * 360.0;
	return frc::InputModulus(degrees, -180.0, 180.0);
}

double SuperStructure::getUpperAngleFalcon() {
	double rawUpperEncoder = upperMotor.GetPosition().GetValueAsDouble(); // Goes from 0 to 1
	double degrees = rawUpperEncoder * 360.0;
	return -frc::InputModulus(degrees, -180.0, 180.0);
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

	lowerLeftMotor.setMotionMagicPosition(convertAngleToFalconPos(actualTarget.lowerAngle), 0.0, false);

	double voltageUpperOut = upperPID.Calculate(units::degree_t(currentState.upperAngle), units::degree_t(actualTarget.upperAngle));
	const auto upperSetpoint = upperPID.GetSetpoint();
	// voltageUpperOut += upperFF.Calculate(lowerSetpoint.position + upperSetpoint.position + upperFFOffset, upperSetpoint.velocity).value();

	upperMotor.SetVoltage(units::volt_t(voltageUpperOut));
}

void SuperStructure::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Lower", currentState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Upper", currentState.upperAngle);

	frc::SmartDashboard::PutNumber("SuperStructure/Current/RawLower", lowerCANCoder.getSensorAbsolutePosition());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/RawUpper", upperEncoder.GetAbsolutePosition());


	frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Lower", targetState.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Upper", targetState.upperAngle);

	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Lower", actualTarget.lowerAngle);
	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Upper", actualTarget.upperAngle);

}