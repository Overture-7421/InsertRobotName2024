// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/filter/Debouncer.h>
#include <rev/ColorSensorV3.h>
#include <rev/Rev2mDistanceSensor.h>
#include <units/length.h>

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"

#include "Constants.h"

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>

#include <frc/SerialPort.h>


#define BUFFER_SIZE 255
class Storage : public frc2::SubsystemBase {
public:
	Storage();
	void setVoltage(units::volt_t voltage);
	bool isNoteOnForwardSensor();
	bool isSensorAvailable();
	void Periodic() override;
	void shuffleboardPeriodic();
private:
	OverTalonFX storageMotor{ 30, ControllerNeutralMode::Brake, false, "rio" };
	// rev::ColorSensorV3 colorSensor{frc::I2C::Port::kMXP};
	// int IRvalue = 0;
	frc::DigitalInput beamBreakSensor {5};
	// units::volt_t lastVolts;
	// units::millimeter_t lastRange;
	// units::second_t timeLastReading, timeSinceLastReading;
	// rev::Rev2mDistanceSensor distanceSensor {rev::Rev2mDistanceSensor::Port::kMXP, rev::Rev2mDistanceSensor::DistanceUnit::kMilliMeters, rev::Rev2mDistanceSensor::RangeProfile::kHighSpeed};

	
	// char buffer[BUFFER_SIZE] = {0};
	// frc::SerialPort distanceSensorPort {9600, frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_Even, frc::SerialPort::kStopBits_Two};

	// bool isDistanceSensorConnected = true;

	wpi::log::DataLog& log = frc::DataLogManager::GetLog();
	wpi::log::BooleanLogEntry noteOnForward = wpi::log::BooleanLogEntry(log, "/storage/note_on_forward");
	wpi::log::DoubleLogEntry voltage = wpi::log::DoubleLogEntry(log, "/storage/voltage");
	wpi::log::DoubleLogEntry current = wpi::log::DoubleLogEntry(log, "/storage/current");
	wpi::log::DoubleLogEntry distance = wpi::log::DoubleLogEntry(log, "/storage/distance_sensor");

};
