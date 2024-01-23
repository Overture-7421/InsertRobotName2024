// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
	upperMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	upperMotor.setSensorToMechanism(UPPER_GEAR_BOX_REDUCTION);
	upperMotor.setClosedLoopVoltageRamp(0.5);

	lowerMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	lowerMotor.setSensorToMechanism(LOWER_GEAR_BOX_REDUCTION);
	lowerMotor.setFollow(upperMotor.GetDeviceID(), false);

	upperMotor.setPIDValues(4.0, 0.0, 0.0, 0.0, 0.0);

	//this->velocity = velocity;
}

void Shooter::setVelocityVoltage(double velocity) {
	upperMotor.setVelocityVoltage(velocity, false);
	this->velocity = velocity;

}

// This method will be called once per scheduler run
void Shooter::Periodic() {
	frc::SmartDashboard::PutNumber("Shooter Velocity:", velocity);

}
