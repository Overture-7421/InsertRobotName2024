// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "SuperStructureState.h"
#include "SuperStructurePosition.h"
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h>
#include <OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h>


class SuperStructure : public frc2::SubsystemBase {
public:
	SuperStructure();
	void setTargetCoord(SuperStructureState TargetCoord);
	double getLowerAngle();
	double getUpperAngle();
	SuperStructurePosition getPosition();
	void setPosition(SuperStructurePosition pos);
	SuperStructureState getCurrentState();
	void Periodic() override;

private:
	void setFalconTargetPos(SuperStructureState targetState);
	double convertAngleToFalconPos(double angle);
	double upperAngleFFCalculation(double angle);

	//constant
	const double LOWER_GEAR_BOX_REDUCTION = 251;
	const double UPPER_GEAR_BOX_REDUCTION = 150;

	//Encoders
	OverDutyCycleEncoder lowerEncoder{ 0 };
	OverDutyCycleEncoder upperEncoder{ 1 };
	double lowerOffset = 0;
	double upperOffset = 0;

	// LowerMotors
	OverTalonFX m_lowerRight{ 20, ControllerNeutralMode::Brake, true, "rio" };
	OverTalonFX m_lowerLeft{ 21, ControllerNeutralMode::Brake, true, "rio" };

	// Upper Motors
	OverTalonFX m_upperMotor{ 22, ControllerNeutralMode::Brake, true, "rio" };

	// State
	SuperStructureState m_TargetState{ getCurrentState() };
	SuperStructurePosition position = SuperStructurePosition::Closing;

	//Motion Magic Feed Forward
	double lowerFF = 0.0;
	double upperFF = 0.0;
};
