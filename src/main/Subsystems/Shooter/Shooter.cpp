// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
	shooterMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	shooterMotor.setSensorToMechanism(UPPER_GEAR_BOX_REDUCTION);
	shooterMotor.setClosedLoopVoltageRamp(0.5);

	shooterMotor.setPIDValues(0.00055, 0.0, 0.0, 0.0, 0.0495);
}

void Shooter::setVelocityVoltage(double velocity) {
	shooterMotor.setVelocityVoltage(velocity, false);
	this->velocity = velocity;
}

double Shooter::getCurrentVelocity() {
	return shooterMotor.GetVelocity().GetValue().value();
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
	frc::SmartDashboard::PutNumber("Shooter/VelocityTarget", velocity);
	frc::SmartDashboard::PutNumber("Shooter/Velocity", getCurrentVelocity());
}
