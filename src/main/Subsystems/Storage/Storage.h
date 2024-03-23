// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/filter/Debouncer.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"

#include "Constants.h"

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>

class Storage : public frc2::SubsystemBase {
public:
	Storage();
	void setVoltage(units::volt_t voltage);
	bool isNoteOnForwardSensor();
	bool isNoteOnBackSensor();
	void Periodic() override;
	void shuffleboardPeriodic();
private:
	OverTalonFX storageMotor{ 24, ControllerNeutralMode::Brake, false, "rio" };

	frc::DigitalInput forwardSensor{ 0 }; //2
	frc::DigitalInput backSensor{ 2 }; // 0

	frc::Debouncer m_debouncer{ 35_ms, frc::Debouncer::DebounceType::kBoth };
	bool noteOnForwardCache = false;

	wpi::log::DataLog& log = frc::DataLogManager::GetLog();
	wpi::log::BooleanLogEntry noteOnForward = wpi::log::BooleanLogEntry(log, "/storage/note_on_forward");
	wpi::log::DoubleLogEntry voltage = wpi::log::DoubleLogEntry(log, "/storage/voltage");
	wpi::log::DoubleLogEntry current = wpi::log::DoubleLogEntry(log, "/storage/current");

};
