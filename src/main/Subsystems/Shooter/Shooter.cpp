// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
	upperShooterMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	upperShooterMotor.setSensorToMechanism(UPPER_GEAR_BOX_REDUCTION);
	upperShooterMotor.setClosedLoopVoltageRamp(0.5);

	lowerShooterMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	lowerShooterMotor.setSensorToMechanism(UPPER_GEAR_BOX_REDUCTION);
	lowerShooterMotor.setClosedLoopVoltageRamp(0.5);

	upperShooterMotor.setPIDValues(4.0, 0.0, 0.0, 0.0, 0.0);
	lowerShooterMotor.setPIDValues(4.0, 0.0, 0.0, 0.0, 0.0);

	//this->velocity = velocity;
}

void Shooter::setVelocityVoltage(double velocity) {
	upperShooterMotor.setVelocityVoltage(velocity, false);
	lowerShooterMotor.setVelocityVoltage(velocity, false);
	this->velocity = velocity;
}

void Shooter::setVoltage(units::volt_t voltage) {
	upperShooterMotor.setVoltage(voltage, false);
	lowerShooterMotor.setVoltage(voltage, false);

}

double Shooter::getUpperMotorCurrentVelocity() {
	return upperShooterMotor.getVelocity(1);

}

double Shooter::getLowerMotorCurrentVelocity() {
	return lowerShooterMotor.getVelocity(1);

}


// This method will be called once per scheduler run
void Shooter::Periodic() {
	frc::SmartDashboard::PutNumber("Shooter Velocity Goal:", velocity);
	frc::SmartDashboard::PutNumber("Shooter Upper Velocity:", getUpperMotorCurrentVelocity());
	frc::SmartDashboard::PutNumber("Shooter Lower Velocity:", getLowerMotorCurrentVelocity());

}
