// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"

class Shooter : public frc2::SubsystemBase {
public:
	Shooter();
	void setVoltage(units::volt_t voltage);
	void Periodic() override;

private:
	OverTalonFX upperMotor{ 26, ControllerNeutralMode::Brake, false, "rio" };
	OverTalonFX lowerMotor{ 27, ControllerNeutralMode::Brake, false, "rio" };

};
