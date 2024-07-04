// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Storage.h"
#include <iostream>

Storage::Storage() {
	storageMotor.setSupplyCurrentLimit(true, 20, 30, 0.5);
	// distanceSensorL.SetAutomaticMode(true);
	// distanceSensorR.SetAutomaticMode(true);
}

void Storage::setVoltage(units::volt_t voltage) {
	storageMotor.setVoltage(voltage, true);
}

bool Storage::isNoteOnForwardSensor() {
	if(isSensorAvailable()) {
		return isNoteOnStorage;
	}
	return false;
}

bool Storage::isSensorAvailable() {
	return true;
}

void Storage::Periodic() {
	// const auto currentTime = frc::Timer::GetFPGATimestamp();
	// const auto currentRangeL = distanceSensorL.GetRange();
	// const auto currentRangeR = distanceSensorR.GetRange();

	// if(currentRangeL > 1_cm) {
	// 	if(currentRangeL != lastRangeL) {
	// 		timeLastReading = currentTime;
	// 	}
	// 	lastRangeL = currentRangeL;
	// }

	// if(currentRangeR > 1_cm) {
	// 	if(currentRangeR != lastRangeR) {
	// 		timeLastReading = currentTime;
	// 	}
	// 	lastRangeR = currentRangeR;
	// }

	// timeSinceLastReading = currentTime - timeLastReading;
	// isDistanceSensorConnected = timeSinceLastReading < StorageConstants::DistanceSensorAvailableTimeTolerance;
	// isNoteOnStorage = lastRangeL < StorageConstants::DistanceSensorActivationThreshold || lastRangeR < StorageConstants::DistanceSensorActivationThreshold;
	beamBreak1Cache = !beamBreak1.Get();
	beamBreak2Cache = !beamBreak2.Get();
	isNoteOnStorage = beamBreak1Cache || beamBreak2Cache;
}

frc2::CommandPtr Storage::storageCommand(units::volt_t voltage){
	return frc2::cmd::RunOnce([this, voltage] {this->setVoltage(voltage);});
}

void Storage::shuffleboardPeriodic() {
	noteOnForward.Append(isNoteOnForwardSensor());
	voltage.Append(storageMotor.GetMotorVoltage().GetValueAsDouble());
	current.Append(storageMotor.GetSupplyCurrent().GetValueAsDouble());
	beamBreak1Log.Append(beamBreak1Cache);
	beamBreak2Log.Append(beamBreak2Cache);
	// distanceL.Append(lastRangeL.value());
	// distanceR.Append(lastRangeR.value());
	// sensorAvailable.Append(isSensorAvailable());
	frc::SmartDashboard::PutBoolean("Storage/NoteOnForward", isNoteOnForwardSensor());
	frc::SmartDashboard::PutBoolean("Storage/BeamBreak1", beamBreak1Cache);
	frc::SmartDashboard::PutBoolean("Storage/BeamBreak2", beamBreak2Cache);

	// frc::SmartDashboard::PutNumber("Storage/DistanceSensorL", lastRangeL.value());
	// frc::SmartDashboard::PutNumber("Storage/DistanceSensorR", lastRangeR.value());
}
