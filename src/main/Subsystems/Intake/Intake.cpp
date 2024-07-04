// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
	intakeMotorLeft.setSupplyCurrentLimit(true, 20, 30, 0.5);
}

void Intake::setVoltage(units::volt_t voltage) {
	intakeMotorLeft.setVoltage(voltage, true);
}

frc2::CommandPtr Intake::startIntake(){
	return frc2::cmd::RunOnce([this] { this->setVoltage(IntakeConstants::GroundGrabVolts);});
}

frc2::CommandPtr Intake::stopIntake(){
	return frc2::cmd::RunOnce([this] { this->setVoltage(IntakeConstants::StopVolts);});
}

frc2::CommandPtr Intake::reverseIntake(){
	return frc2::cmd::RunOnce([this] { this->setVoltage(IntakeConstants::ReverseVolts);});
}

double Intake::getVoltage(){
	return intakeMotorLeft.GetMotorVoltage().GetValueAsDouble();
}

// This method will be called once per scheduler run
void Intake::Periodic() {}


void Intake::shuffleboardPeriodic() {
	voltage.Append(intakeMotorLeft.GetMotorVoltage().GetValueAsDouble());
	currentLeft.Append(intakeMotorLeft.GetSupplyCurrent().GetValueAsDouble());

	frc::SmartDashboard::PutNumber("Intake/CurrentLeft", intakeMotorLeft.GetSupplyCurrent().GetValueAsDouble());
	
}