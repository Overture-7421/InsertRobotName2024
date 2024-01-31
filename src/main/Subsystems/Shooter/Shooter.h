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
	void setVelocityVoltage(double velocity);
	double getCurrentVelocity();
	void Periodic() override;

private:
	OverTalonFX shooterMotor{ 26, ControllerNeutralMode::Brake, false, "rio" };

	const double LOWER_GEAR_BOX_REDUCTION = 1.0/2.4;
	const double UPPER_GEAR_BOX_REDUCTION = 1.0/2.4;

	double velocity;
};
