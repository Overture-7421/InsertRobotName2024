// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Storage.h"

Storage::Storage() {
	storageMotor.setSupplyCurrentLimit(true, 20, 30, 0.5);

	distanceSensor.SetAutomaticMode(true);
	distanceSensor.SetEnabled(true);
}

void Storage::setVoltage(units::volt_t voltage) {
	storageMotor.setVoltage(voltage, false);
}

bool Storage::isNoteOnForwardSensor() {
	// return lastRange < StorageConstants::DistanceSensorActivationThreshold;
	return false;
}

bool Storage::isNoteOnBackSensor(){
	return false;
}

// This method will be called once per scheduler run
void Storage::Periodic() {
	auto currentRange = units::millimeter_t(distanceSensor.GetRange());
	if(currentRange > 0_m) {
		lastRange = currentRange;
	}
}

void Storage::shuffleboardPeriodic() {
	noteOnForward.Append(isNoteOnForwardSensor());
	voltage.Append(storageMotor.GetMotorVoltage().GetValueAsDouble());
	current.Append(storageMotor.GetSupplyCurrent().GetValueAsDouble());
	distance.Append(lastRange.value());
	frc::SmartDashboard::PutBoolean("Storage/NoteOnForward", isNoteOnForwardSensor());
	frc::SmartDashboard::PutNumber("Storage/DistanceSensor", lastRange.value() / 10.0);
}
