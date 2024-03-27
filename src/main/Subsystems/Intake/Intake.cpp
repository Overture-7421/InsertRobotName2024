// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
	intakeMotorLeft.setSupplyCurrentLimit(true, 20, 30, 0.5);
	intakeMotorRight.setFollow(25, true);
}

void Intake::setVoltage(units::volt_t voltage) {
	intakeMotorLeft.setVoltage(voltage, false);
}

// This method will be called once per scheduler run
void Intake::Periodic() {}


void Intake::shuffleboardPeriodic() {
	voltage.Append(intakeMotorLeft.GetMotorVoltage().GetValueAsDouble());
	current.Append(intakeMotorLeft.GetSupplyCurrent().GetValueAsDouble());
}