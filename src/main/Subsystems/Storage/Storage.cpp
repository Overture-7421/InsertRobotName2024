// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Storage.h"
#include <iostream>

Storage::Storage() {
	storageMotor.setSupplyCurrentLimit(true, 20, 30, 0.5);
	distanceSensor.SetAutomaticMode(true);
}

void Storage::setVoltage(units::volt_t voltage) {
	storageMotor.setVoltage(voltage, false);
}

bool Storage::isNoteOnForwardSensor() {
	if(isSensorAvailable()) {
		return lastRange < StorageConstants::DistanceSensorActivationThreshold;
	}
	return false;
}

bool Storage::isSensorAvailable() {
	return isDistanceSensorConnected;
}

void Storage::Periodic() {
	const auto currentTime = frc::Timer::GetFPGATimestamp();
	const auto currentRange = distanceSensor.GetRange();

	if(lastRange > 0_cm && currentRange != lastRange) {
		timeLastReading = currentTime;
	}
	lastRange = currentRange;

	timeSinceLastReading = currentTime - timeLastReading;
	isDistanceSensorConnected = timeSinceLastReading < StorageConstants::DistanceSensorAvailableTimeTolerance;
}

void Storage::shuffleboardPeriodic() {
	noteOnForward.Append(isNoteOnForwardSensor());
	voltage.Append(storageMotor.GetMotorVoltage().GetValueAsDouble());
	current.Append(storageMotor.GetSupplyCurrent().GetValueAsDouble());
	distance.Append(lastRange.value());
	sensorAvailable.Append(isSensorAvailable());
	frc::SmartDashboard::PutBoolean("Storage/NoteOnForward", isNoteOnForwardSensor());
	frc::SmartDashboard::PutNumber("Storage/DistanceSensor", lastRange.value() * 100);
}
