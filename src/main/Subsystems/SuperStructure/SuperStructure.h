// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h>
#include <OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h>
#include "Constants.h"
#include "SuperStructureState.h"

class SuperStructure : public frc2::SubsystemBase {
public:
	SuperStructure();
	void setTargetCoord(SuperStructureState targetState);
	double getLowerAngle();
	double getUpperAngle();
	SuperStructureState getCurrentState();
	void Periodic() override;

private:
	void setFalconTargetPos(SuperStructureState targetState, SuperStructureState currentState);
	double convertAngleToFalconPos(double angle);

	//Encoders
	OverDutyCycleEncoder lowerEncoder{ 0 };
	OverDutyCycleEncoder upperEncoder{ 1 };
	double lowerOffset = 0;
	double upperOffset = 0;

	// LowerMotors
	OverTalonFX lowerRightMotor{ 20, ControllerNeutralMode::Brake, true, "rio" };
	OverTalonFX lowerLeftMotor{ 21, ControllerNeutralMode::Brake, true, "rio" };

	// Upper Motors
	OverTalonFX upperMotor{ 22, ControllerNeutralMode::Brake, true, "rio" };

	// State
	SuperStructureState targetState;

	//Motion Magic Feed Forward
	double lowerFF = 0.25;
	double upperFF = 0.1;
};
