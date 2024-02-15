// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
	upperShooterMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	upperShooterMotor.setSensorToMechanism(ShooterConstants::LowerGearboxReduction);
	upperShooterMotor.setClosedLoopVoltageRamp(0.5);

	lowerShooterMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);
	lowerShooterMotor.setSensorToMechanism(ShooterConstants::UpperGearboxReduction);
	lowerShooterMotor.setClosedLoopVoltageRamp(0.5);

	upperShooterMotor.setPIDValues(0.010558, 0.0, 0.0, 0.0, 0.0);
	lowerShooterMotor.setPIDValues(0.075634, 0.0, 0.0, 0.0, 0.0);
}

void Shooter::setVelocityVoltage(double velocity) {
	targetVel = velocity;
	upperShooterMotor.setVelocityVoltage(velocity, upperFF.Calculate(units::turns_per_second_t(velocity)).value(), false);
	lowerShooterMotor.setVelocityVoltage(velocity, lowerFF.Calculate(units::turns_per_second_t(velocity)).value(), false);
}

double Shooter::getCurrentVelocity() {
	return (getUpperMotorCurrentVelocity() + getLowerMotorCurrentVelocity()) / 2.0;
}

double Shooter::getUpperMotorCurrentVelocity() {
	return upperShooterMotor.GetVelocity().GetValue().value();

}

double Shooter::getLowerMotorCurrentVelocity() {
	return lowerShooterMotor.GetVelocity().GetValue().value();

}

// This method will be called once per scheduler run
void Shooter::Periodic() {
	frc::SmartDashboard::PutNumber("Shooter/DesiredSpeed", targetVel);
	frc::SmartDashboard::PutNumber("Shooter/CurrentSpeed", getCurrentVelocity());

	frc::SmartDashboard::PutNumber("Shooter/Lower/CurrentSpeed", getLowerMotorCurrentVelocity());
	frc::SmartDashboard::PutNumber("Shooter/Upper/CurrentSpeed", getUpperMotorCurrentVelocity());
}
