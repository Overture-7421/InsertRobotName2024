// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructure.h"
#include <frc/MathUtil.h>
#include <thread>
#include <iostream>

#define DEG_TO_RAD M_PI / 180.0

SuperStructure::SuperStructure() {
	auto lowerConfig = lowerCANCoder.getConfiguration();
	lowerConfig.MagnetSensor.SensorDirection = SensorDirectionValue::Clockwise_Positive;
	lowerCANCoder.GetConfigurator().Apply(lowerConfig);

	auto upperConfig = upperCANCoder.getConfiguration();
	upperConfig.MagnetSensor.SensorDirection = SensorDirectionValue::Clockwise_Positive;
	upperCANCoder.GetConfigurator().Apply(upperConfig);

	// Configure Motors
	lowerLeftMotor.setSensorPosition(lowerCANCoder.getSensorAbsolutePosition());
	upperMotor.setSensorPosition(upperCANCoder.getSensorAbsolutePosition());

	lowerLeftMotor.setFusedCANCoder(lowerCANCoder.GetDeviceID());
	lowerLeftMotor.setSupplyCurrentLimit(true, 40, 60, 1);
	lowerLeftMotor.setRotorToSensorRatio(SuperStructureConstants::LowerAngleGearRatio);

	lowerRightMotor.setSupplyCurrentLimit(true, 40, 60, 1);
	lowerRightMotor.setFollow(lowerLeftMotor.GetDeviceID(), true);

	upperMotor.setFusedCANCoder(upperCANCoder.GetDeviceID());
	upperMotor.setSupplyCurrentLimit(true, 40, 60, 1);
	upperMotor.setRotorToSensorRatio(SuperStructureConstants::UpperAngleGearRatio);

	lowerLeftMotor.setClosedLoopVoltageRamp(0.01);
	upperMotor.setClosedLoopVoltageRamp(0.01);

	SoftwareLimitSwitchConfigs lowerMotorSoftLimitConfig;
	lowerMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ForwardSoftLimitThreshold = units::turn_t{ SuperStructureConstants::LowerAngleUpperLimit }.value();

	lowerMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	lowerMotorSoftLimitConfig.ReverseSoftLimitThreshold = units::turn_t{ SuperStructureConstants::LowerAngleLowerLimit }.value();

	lowerLeftMotor.configureSoftwareLimitSwitch(lowerMotorSoftLimitConfig);

	SoftwareLimitSwitchConfigs upperMotorSoftLimitConfig;
	upperMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ForwardSoftLimitThreshold = units::turn_t{ SuperStructureConstants::UpperAngleUpperLimit }.value();

	upperMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
	upperMotorSoftLimitConfig.ReverseSoftLimitThreshold = units::turn_t{ SuperStructureConstants::UpperAngleLowerLimit }.value();

	upperMotor.configureSoftwareLimitSwitch(upperMotorSoftLimitConfig);

	setTargetCoord({ lowerCANCoder.GetAbsolutePosition().GetValue(), upperCANCoder.GetAbsolutePosition().GetValue() });

	// Configure Motion Magic and PID
	lowerLeftMotor.setPIDValues(100, 10, 0.0, 0.0, 0.0);
	lowerLeftMotor.configureMotionMagic(1.25, 5.0, 0.0);

	upperMotor.setPIDValues(100.0, 60.0, 0.0, 0.0, 0.0);
	upperMotor.configureMotionMagic(1.5, 2, 0.0);

}

void SuperStructure::setTargetCoord(SuperStructureState targetState) {
	this->targetState = targetState;
}

units::degree_t SuperStructure::getLowerAngle() {
	return lowerLeftMotor.GetPosition().GetValue();
}

units::degree_t SuperStructure::getUpperAngle() {
	return upperMotor.GetPosition().GetValue();
}

SuperStructureState SuperStructure::getCurrentState() {
	SuperStructureState state;
	state.lowerAngle = getLowerAngle();
	state.upperAngle = getUpperAngle();
	return state;
}

bool SuperStructure::reachedTargetPosition(SuperStructureState targetState) {
	double lowError = abs((targetState.lowerAngle - getLowerAngle()).value());
	double upperError = abs((targetState.upperAngle - getUpperAngle()).value());

	if (lowError <= 5 && upperError <= 5) {
		return true;
	} else {
		return false;
	}
}

frc2::CommandPtr SuperStructure::superStructureCommand(SuperStructureState targetState) {
	return frc2::FunctionalCommand(
		[&, targetState]() {setTargetCoord(targetState);},
		[&]() {},
		[&](bool interupted) {},
		[&, targetState]() {return reachedTargetPosition(targetState);},
		{ this }
	).ToPtr();
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

	auto turns = units::turn_t(lowerLeftMotor.GetClosedLoopReference().GetValueAsDouble());
	auto turnsPerSecond = units::turns_per_second_t(lowerRightMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

	auto wristTurns = units::turn_t(upperMotor.GetClosedLoopReference().GetValueAsDouble());
	auto wristTurnsPerSecond = units::turns_per_second_t(upperMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

	lowerLeftMotor.setMotionMagicPosition(units::turn_t{ actualTarget.lowerAngle }.value(), lowerFeedForward.Calculate(turns, turnsPerSecond).value(), true);
	upperMotor.setMotionMagicPosition(units::turn_t{ actualTarget.upperAngle }.value(), upperFeedForward.Calculate(wristTurns, wristTurnsPerSecond).value(), true);
}

void SuperStructure::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Lower", currentState.lowerAngle.value());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/Upper", currentState.upperAngle.value());

	frc::SmartDashboard::PutNumber("SuperStructure/Current/RawLower", lowerCANCoder.getSensorAbsolutePosition());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/LowerMotorPosition", lowerLeftMotor.GetPosition().GetValue().value());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/RawUpper", upperCANCoder.getSensorAbsolutePosition());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/UpperMotorPosition", upperMotor.GetPosition().GetValue().value());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/RawLower", lowerCANCoder.getSensorAbsolutePosition());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/LowerMotorPosition", lowerLeftMotor.GetPosition().GetValue().value());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/RawUpper", upperCANCoder.getSensorAbsolutePosition());
	frc::SmartDashboard::PutNumber("SuperStructure/Current/UpperMotorPosition", upperMotor.GetPosition().GetValue().value());


	//frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Lower", targetState.lowerAngle);
	//frc::SmartDashboard::PutNumber("SuperStructure/DesiredTarget/Upper", targetState.upperAngle);

	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Lower", actualTarget.lowerAngle.value());
	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/LowerMotor", units::turn_t{ actualTarget.lowerAngle }.value());

	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/Upper", actualTarget.upperAngle.value());
	frc::SmartDashboard::PutNumber("SuperStructure/ActualTarget/UpperMotor", units::turn_t{ actualTarget.upperAngle }.value());

}
