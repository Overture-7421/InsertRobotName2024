// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ArmFeedforward.h>

#include "SuperStructureState.h"
#include "SuperStructurePosition.h"
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h>
#include <OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h>
#include <frc2/command/sysid/SysIdRoutine.h>


class SuperStructure : public frc2::SubsystemBase {
public:
	SuperStructure();
	void setTargetCoord(SuperStructureState TargetCoord);
	void setLowerAngleConstraints(double velocity, double acceleration);
	void setUpperAngleConstraints(double velocity, double acceleration);
	double getLowerAngle();
	double getUpperAngle();

	frc2::CommandPtr sysIdQuadstaticLower(frc2::sysid::Direction direction) {
		return sysIdRoutineLower.Quasistatic(direction);
	}

	frc2::CommandPtr sysIdDinamicLower(frc2::sysid::Direction direction) {
		return sysIdRoutineLower.Dynamic(direction);
	}

	frc2::CommandPtr sysIdQuadstaticUpper(frc2::sysid::Direction direction) {
		return sysIdRoutineUpper.Quasistatic(direction);
	}

	frc2::CommandPtr sysIdDinamicUpper(frc2::sysid::Direction direction) {
		return sysIdRoutineUpper.Dynamic(direction);
	}

	SuperStructurePosition getPosition();
	void setPosition(SuperStructurePosition pos);
	SuperStructureState getCurrentState();
	void Periodic() override;


private:
	void setFalconTargetPos(SuperStructureState targetState, SuperStructureState currentState);
	double convertAngleToFalconPos(double angle);

	//constant
	const double LOWER_GEAR_BOX_REDUCTION = 230.4;
	const double UPPER_GEAR_BOX_REDUCTION = 90.0;

	//Encoders
	OverDutyCycleEncoder lowerEncoder{ 3 };
	OverDutyCycleEncoder upperEncoder{ 2 };
	double lowerOffset = -0.338472;
	double upperOffset = -0.577487;


	// LowerMotors
	OverTalonFX m_lowerRight{ 20, ControllerNeutralMode::Coast, false, "rio" };
	OverTalonFX m_lowerLeft{ 21, ControllerNeutralMode::Coast, false, "rio" };

	// Upper Motors
	OverTalonFX m_upperMotor{ 22, ControllerNeutralMode::Coast, true, "rio" };

	// State
	SuperStructureState m_TargetState{ getCurrentState() };
	SuperStructurePosition position = SuperStructurePosition::Closing;

	//Motion Magic Feed Forward
	double lowerFF = 2.75;
	double upperFF = 0.5;

	frc2::sysid::SysIdRoutine sysIdRoutineLower{
		frc2::sysid::Config{0.75_V / 1_s, 4_V, 3_s,
							std::nullopt},
		frc2::sysid::Mechanism{
			[this](units::volt_t driveVoltage) {
				m_lowerRight.SetVoltage(driveVoltage);
			},
			[this](frc::sysid::SysIdRoutineLog* log) {

			log->Motor("SuperStructureLower")
				.voltage(m_lowerRight.GetMotorVoltage().GetValue())
				.position(m_lowerRight.GetPosition().GetValue())
				.velocity(m_lowerRight.GetVelocity().GetValue());
			},
			this} };

	frc2::sysid::SysIdRoutine sysIdRoutineUpper{
		frc2::sysid::Config{0.75_V / 1_s, 4_V, 4_s,
							std::nullopt},
		frc2::sysid::Mechanism{
			[this](units::volt_t driveVoltage) {
				m_lowerRight.SetVoltage(driveVoltage);
			},
			[this](frc::sysid::SysIdRoutineLog* log) {

			log->Motor("SuperStructureUpper")
				.voltage(m_upperMotor.GetMotorVoltage().GetValue())
				.position(m_upperMotor.GetPosition().GetValue())
				.velocity(m_upperMotor.GetVelocity().GetValue());
			},
			this} };
};
