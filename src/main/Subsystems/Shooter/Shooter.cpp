// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
	ShooterMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	ShooterMotor.setSensorToMechanism(UPPER_GEAR_BOX_REDUCTION);
	ShooterMotor.setClosedLoopVoltageRamp(0.5);

	ShooterMotor.setPIDValues(4.0, 0.0, 0.0, 0.0, 0.0);

	//this->velocity = velocity;
}

void Shooter::setVelocityVoltage(double velocity) {
	ShooterMotor.setVelocityVoltage(velocity, false);
	this->velocity = velocity;

}


void Shooter::setVoltage(units::volt_t voltage) {
	ShooterMotor.setVoltage(voltage, false);
}


double Shooter::getCurrentVelocity() {
	return ShooterMotor.getVelocity(1);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
	frc::SmartDashboard::PutNumber("Shooter Velocity Goal:", velocity);
	frc::SmartDashboard::PutNumber("Shooter Velocity:", getCurrentVelocity());
}
