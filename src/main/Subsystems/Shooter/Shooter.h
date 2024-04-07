// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "MotorControllers/OverTalonFX/OverTalonFX.h"
#include "MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
public:
	Shooter();
	void setTargetVelocity(double velocity);
	void setVoltage(double voltage);
	void setEmergencyDisable(bool emergencyDisable);
	bool isEmergencyDisabled();
	double getCurrentVelocity();
	void Periodic() override;
	void shuffleboardPeriodic();

	frc2::CommandPtr sysIdQuasistatic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Quasistatic(direction);
	}

	frc2::CommandPtr sysIdDynamic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Dynamic(direction);
	}

private:
	OverTalonFX leftShooterMotor{ 27, ControllerNeutralMode::Coast, false, "rio" };
	OverTalonFX rightShooterMotor{ 26, ControllerNeutralMode::Coast, false, "rio" };

	frc::SimpleMotorFeedforward<units::turn> shooterFF {0.17356_V, 0.067254_V / 1_tps, 0.016758_V / 1_tr_per_s_sq};

	double targetVel = 0.0;

	bool emergencyDisabled = false;

	frc2::sysid::SysIdRoutine sysIdRoutine{
	frc2::sysid::Config{1_V / 1_s, 7_V, 10_s,
						std::nullopt},
	frc2::sysid::Mechanism{
		[this](units::volt_t driveVoltage) {
			leftShooterMotor.SetVoltage(driveVoltage);
		},
		[this](frc::sysid::SysIdRoutineLog* log) {

		log->Motor("Shooter")
			.voltage(leftShooterMotor.GetMotorVoltage().GetValue())
			.position(leftShooterMotor.GetPosition().GetValue())
			.velocity(leftShooterMotor.GetVelocity().GetValue());
		},
		this} };
};
