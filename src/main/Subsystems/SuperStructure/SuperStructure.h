// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ArmFeedforward.h>

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h>
#include <OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "Constants.h"
#include "SuperStructureState.h"

class SuperStructure : public frc2::SubsystemBase {
public:
	SuperStructure();
	void setTargetCoord(SuperStructureState targetState);
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

	SuperStructureState getCurrentState();
	void Periodic() override;


private:
	void setFalconTargetPos(SuperStructureState targetState, SuperStructureState currentState);
	double convertAngleToFalconPos(double angle);

	//Encoders
	OverDutyCycleEncoder lowerEncoder{ 3 };
	OverDutyCycleEncoder upperEncoder{ 2 };
	double lowerOffset = -0.338472;
	double upperOffset = -0.577487;


	// LowerMotors
	OverTalonFX lowerRightMotor{ 20, ControllerNeutralMode::Brake, true, "rio" };
	OverTalonFX lowerLeftMotor{ 21, ControllerNeutralMode::Brake, true, "rio" };

	// Upper Motors
	OverTalonFX upperMotor{ 22, ControllerNeutralMode::Brake, true, "rio" };

	// State
	SuperStructureState targetState;

	//Motion Magic Feed Forward
	double lowerFF = 2.75;
	double upperFF = 0.5;

	frc2::sysid::SysIdRoutine sysIdRoutineLower{
		frc2::sysid::Config{0.75_V / 1_s, 4_V, 3_s,
							std::nullopt},
		frc2::sysid::Mechanism{
			[this](units::volt_t driveVoltage) {
				lowerRightMotor.SetVoltage(driveVoltage);
			},
			[this](frc::sysid::SysIdRoutineLog* log) {

			log->Motor("SuperStructureLower")
				.voltage(lowerRightMotor.GetMotorVoltage().GetValue())
				.position(lowerRightMotor.GetPosition().GetValue())
				.velocity(lowerRightMotor.GetVelocity().GetValue());
			},
			this} };

	frc2::sysid::SysIdRoutine sysIdRoutineUpper{
		frc2::sysid::Config{0.75_V / 1_s, 4_V, 4_s,
							std::nullopt},
		frc2::sysid::Mechanism{
			[this](units::volt_t driveVoltage) {
				upperMotor.SetVoltage(driveVoltage);
			},
			[this](frc::sysid::SysIdRoutineLog* log) {

			log->Motor("SuperStructureUpper")
				.voltage(upperMotor.GetMotorVoltage().GetValue())
				.position(upperMotor.GetPosition().GetValue())
				.velocity(upperMotor.GetVelocity().GetValue());
			},
			this} };
};
