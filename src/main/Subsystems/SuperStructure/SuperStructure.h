// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/Sensors/OverCANCoder/OverCANCoder.h>
#include <OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h>
#include <OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/Commands.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>

#include <numbers>
#include <frc/estimator/KalmanFilter.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"
#include "SuperStructureState.h"
#include <OvertureLib/Robots/OverRobot/RobotConstants.h>

#include <frc2/command/Commands.h>

//Offset Lower  -0.32608
//Offset Upper -0.21753

class SuperStructure : public frc2::SubsystemBase {
public:
	SuperStructure();
	void setTargetCoord(SuperStructureState targetState);
	double getLowerAngle();
	double getUpperAngle();
	bool reachedTargetPosition(SuperStructureState targetState);
	frc2::CommandPtr superStructureCommand(SuperStructureState targetState);

	void setArbitraryFeedForwardUpper(units::volt_t feedforward);

	frc2::CommandPtr sysIdQuasistaticLower(frc2::sysid::Direction direction) {
		return sysIdRoutineLower.Quasistatic(direction);
	}

	frc2::CommandPtr sysIdDynamicLower(frc2::sysid::Direction direction) {
		return sysIdRoutineLower.Dynamic(direction);
	}

	frc2::CommandPtr sysIdQuasistaticUpper(frc2::sysid::Direction direction) {
		return frc2::cmd::Sequence(
			frc2::cmd::Run([this] {setTargetCoord({ 0, 0 });}).WithTimeout(1.5_s),
			sysIdRoutineUpper.Quasistatic(direction)
		);
	}

	frc2::CommandPtr sysIdDynamicUpper(frc2::sysid::Direction direction) {
		return frc2::cmd::Sequence(
			frc2::cmd::Run([this] {setTargetCoord({ 0, 0 });}).WithTimeout(1.5_s),
			sysIdRoutineUpper.Dynamic(direction)
		);
	}

	SuperStructureState getCurrentState();
	void Periodic() override;
	void shuffleboardPeriodic();

private:
	double convertAngleToFalconPos(double angle);

	units::volt_t arbitraryFeedForwardUpper = 0_V;
	// LowerMotors
	OverTalonFX lowerRightMotor{ 21, ControllerNeutralMode::Brake, false, "rio" };
	OverTalonFX lowerLeftMotor{ 22, ControllerNeutralMode::Brake, true, "rio" };

	// Upper Motors
	OverTalonFX upperMotor{ 23, ControllerNeutralMode::Brake, true, "rio" };

	// Encoders
	OverCANCoder lowerCANCoder{ 28, -0.3132_tr, "rio" };
	OverCANCoder upperCANCoder{ 27, 0.2553_tr + 0.0085_tr , "rio" };

	// State
	SuperStructureState targetState, actualTarget;
	SuperStructureState currentState;

	//Feed Forward
	frc::ArmFeedforward lowerFF{ 1.1646_V, 1.9026_V, 4.1593_V / 1_tps, 3.5982_V / 1_tr_per_s_sq };
	frc::ArmFeedforward upperFF{ 3.2822_V, 0.17171_V, 18.086_V / 1_tps, 3.3682_V / 1_tr_per_s_sq };

	frc2::sysid::SysIdRoutine sysIdRoutineLower{
		frc2::sysid::Config{0.5_V / 1_s, 7_V, 10_s,
							std::nullopt},
		frc2::sysid::Mechanism{
			[this](units::volt_t driveVoltage) {
				lowerLeftMotor.SetVoltage(driveVoltage);
			},
			[this](frc::sysid::SysIdRoutineLog* log) {

			log->Motor("SuperStructureLower")
				.voltage(lowerLeftMotor.GetMotorVoltage().GetValue())
				.position(lowerLeftMotor.GetPosition().GetValue())
				.velocity(lowerLeftMotor.GetVelocity().GetValue());
			},
			this} };

	frc2::sysid::SysIdRoutine sysIdRoutineUpper{
		frc2::sysid::Config{0.25_V / 1_s, 4_V, 10_s,
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
