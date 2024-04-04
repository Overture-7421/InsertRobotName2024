// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Storage.h"
#include <iostream>

Storage::Storage() {
	storageMotor.setSupplyCurrentLimit(true, 20, 30, 0.5);
}

void Storage::setVoltage(units::volt_t voltage) {
	// lastVolts = voltage;
	storageMotor.setVoltage(voltage, false);
}

bool Storage::isNoteOnForwardSensor() {
	// if(isSensorAvailable()) {
	// 	return lastRange < StorageConstants::DistanceSensorActivationThreshold;
	// }
	// return false;
	return beamBreakSensor.Get();
}

bool Storage::isSensorAvailable() {
	return true;
}

void Storage::Periodic() {
	// distanceSensorPort.Read(buffer, BUFFER_SIZE);

	// std::string msg {buffer};

	// int messageEndIndex = -1;
	// int messageStartIndex = -1;

	// for(int i = BUFFER_SIZE - 1; i >= 0 ; i--){
	// 	if(buffer[i] == '\n') {
	// 		if(messageEndIndex == -1){
	// 			messageEndIndex = i;
	// 		}else if(messageStartIndex == -1) {
	// 			messageStartIndex = i;
	// 			break;
	// 		}
	// 	}
	// }

	// // std::string packet = msg.substr(messageStartIndex, messageEndIndex - messageStartIndex);
	// unsigned int range = 0;
	// // std::sscanf(packet.c_str(), "%u\n", &range);

	// std::cout << "--------------------------------------------" << std::endl;
	// std::cout << "Buffer: " << msg << std::endl;
	// std::cout << "Start: " << messageStartIndex << " End: " << messageEndIndex << std::endl;
	// // std::cout << "Msg: " << packet << std::endl;
	// std::cout << "Read value: " << range << std::endl;

	// const auto currentTime = frc::Timer::GetFPGATimestamp();
	// const auto currentRange = units::millimeter_t((double) range);

	// if(lastRange > 0_mm && currentRange != lastRange) {
	// 	timeLastReading = currentTime;
	// }
	// lastRange = currentRange;

	// timeSinceLastReading = currentTime - timeLastReading;
	// isDistanceSensorConnected = timeSinceLastReading < StorageConstants::DistanceSensorAvailableTimeTolerance;
}

void Storage::shuffleboardPeriodic() {
	noteOnForward.Append(isNoteOnForwardSensor());
	voltage.Append(storageMotor.GetMotorVoltage().GetValueAsDouble());
	current.Append(storageMotor.GetSupplyCurrent().GetValueAsDouble());
	// distance.Append(lastRange.value());
	frc::SmartDashboard::PutBoolean("Storage/NoteOnForward", isNoteOnForwardSensor());
	// frc::SmartDashboard::PutNumber("Storage/DistanceSensor", lastRange.value() / /[']10.0);
	frc::SmartDashboard::PutNumber("Storage/Current", storageMotor.GetTorqueCurrent().GetValueAsDouble());

}
