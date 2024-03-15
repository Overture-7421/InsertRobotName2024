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
	frc::DigitalInput forwardSensor{ 2 };
	frc::DigitalInput backSensor{ 0 };
	frc::Debouncer m_debouncer{ 10_ms, frc::Debouncer::DebounceType::kBoth };
};
