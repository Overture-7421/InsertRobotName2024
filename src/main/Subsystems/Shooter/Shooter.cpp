// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
	leftShooterMotor.setSupplyCurrentLimit(true, 40, 60, 0.25);
	leftShooterMotor.setSensorToMechanism(ShooterConstants::GearboxReduction);
	leftShooterMotor.setClosedLoopVoltageRamp(0.1);

	rightShooterMotor.setSupplyCurrentLimit(true, 40, 60, 0.255);

	rightShooterMotor.setFollow(leftShooterMotor.GetDeviceID(), true);

	leftShooterMotor.setPIDValues(0.1, 0, 0, 0, 0.065);
}

void Shooter::setTargetVelocity(double velocity) {
	if (emergencyDisabled) {
		return;
	}

	velocity = std::clamp(velocity, -ShooterConstants::MaxSpeed, ShooterConstants::MaxSpeed);
	targetVel = velocity;
	leftShooterMotor.setVelocityVoltage(velocity, 0, true);
}

void Shooter::setVoltage(double voltage) {
	if (emergencyDisabled && voltage != 0) {
		return;
	}

	leftShooterMotor.SetVoltage(units::volt_t(voltage));
}

void Shooter::setEmergencyDisable(bool emergencyDisable) {
	this->emergencyDisabled = emergencyDisable;
}

bool Shooter::isEmergencyDisabled() {
	return emergencyDisabled;
}

double Shooter::getCurrentVelocity() {
	return leftShooterMotor.GetVelocity().GetValue().value();
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
	if (emergencyDisabled) {
		setVoltage(0);
	}
}

bool Shooter::reachedTargetVelocity(double velocity) {
	double error = std::abs(velocity - getCurrentVelocity());
	if (error < 1) {
		return true;
	}

	return false;
}

frc2::CommandPtr Shooter::shooterCommand(double velocity) {
	return frc2::FunctionalCommand(
		[&, velocity]() {setTargetVelocity(velocity);},
		[&]() {},
		[&](bool interupted) {},
		[&, velocity]() {return reachedTargetVelocity(velocity);},
		{ this }
	).ToPtr();
}

void Shooter::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber("Shooter/DesiredSpeed", targetVel);
	frc::SmartDashboard::PutNumber("Shooter/CurrentSpeed", getCurrentVelocity());
}