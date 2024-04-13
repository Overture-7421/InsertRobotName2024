// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <units/length.h>

#include "MotorControllers/OverTalonFX/OverTalonFX.h"
#include "MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"

#include "Constants.h"

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <frc/Ultrasonic.h>
#include <frc/filter/Debouncer.h>

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
	bool isNoteOnStorage = false, beamBreak1Cache = false, beamBreak2Cache = false;

	frc::DigitalInput beamBreak1 {7};
	frc::DigitalInput beamBreak2 {8};

	wpi::log::DataLog& log = frc::DataLogManager::GetLog();
	wpi::log::BooleanLogEntry noteOnForward = wpi::log::BooleanLogEntry(log, "/storage/note_on_forward");
	wpi::log::BooleanLogEntry beamBreak1Log = wpi::log::BooleanLogEntry(log, "/storage/beam_break_1");
	wpi::log::BooleanLogEntry beamBreak2Log = wpi::log::BooleanLogEntry(log, "/storage/beam_break_2");

	wpi::log::DoubleLogEntry voltage = wpi::log::DoubleLogEntry(log, "/storage/voltage");
	wpi::log::DoubleLogEntry current = wpi::log::DoubleLogEntry(log, "/storage/current");
	// wpi::log::DoubleLogEntry distanceL = wpi::log::DoubleLogEntry(log, "/storage/distance_sensor_left");
	// wpi::log::DoubleLogEntry distanceR = wpi::log::DoubleLogEntry(log, "/storage/distance_sensor_right");
	// wpi::log::BooleanLogEntry sensorAvailable = wpi::log::BooleanLogEntry(log, "/storage/sensor_available");

};
