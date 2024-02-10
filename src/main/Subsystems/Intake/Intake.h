// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"

class Intake : public frc2::SubsystemBase {
public:
	Intake();
	void setVoltage(units::volt_t voltage);
	void Periodic() override;

private:
	OverTalonFX intakeMotor{ 25, ControllerNeutralMode::Brake, false, "rio" };
};
