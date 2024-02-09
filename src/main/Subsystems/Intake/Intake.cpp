// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
	// intakeMotor.setSupplyCurrentLimit(true, 20, 25, 0.5);

	// intakeMotor.setNeutralMode(ControllerNeutralMode::Coast);

	intakeMotor.SetSmartCurrentLimit(20);
	intakeMotor.SetSecondaryCurrentLimit(25);

	intakeMotor.EnableVoltageCompensation(12);
	intakeMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
}

void Intake::setVoltage(units::volt_t voltage) {
	// intakeMotor.setVoltage(voltage, false);

	// intakeMotor.SetVoltage(voltage);
}

// This method will be called once per scheduler run
void Intake::Periodic() {}
