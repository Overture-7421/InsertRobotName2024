// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "SupportArmsState.h"
#include "SupportArmsPosition.h"
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h>
#include <OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h>

class SupportArms : public frc2::SubsystemBase {
public:
	SupportArms();
	void setTargetCoord(SupportArmsState TargetCoord);
	double getLowerAngle();
	SupportArmsPosition getPosition();
	void setPosition(SupportArmsPosition pos);
	SupportArmsState getCurrentState();
	void Periodic() override;

private:
	void setFalconTargetPos(SupportArmsState targetState);
	double convertAngleToFalconPos(double angle);

	//constant
	const double LOWER_GEAR_BOX_REDUCTION = 106;

	//Encoders
	OverDutyCycleEncoder lowerEncoder{ 2 };
	double lowerOffset = 0;


	// LowerMotors
	OverTalonFX m_lowerRight{ 23, ControllerNeutralMode::Brake, true, "rio" };

	// State
	SupportArmsState m_TargetState{ getCurrentState() };
	SupportArmsPosition position = SupportArmsPosition::Closed;

	//Motion Magic Feed Forward
	double lowerFF = 0.0;
};
