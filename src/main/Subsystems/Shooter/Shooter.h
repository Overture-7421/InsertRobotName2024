// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
public:
	Shooter();
	void setVelocityVoltage(double velocity);
	double getCurrentVelocity();
	double getUpperMotorCurrentVelocity();
	double getLowerMotorCurrentVelocity();
	void setVoltage(units::volt_t voltage);
	void Periodic() override;

	frc2::CommandPtr sysIdQuadstatic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Quasistatic(direction);
	}

	frc2::CommandPtr sysIdDinamic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Dynamic(direction);
	}

private:
	OverTalonFX upperShooterMotor{ 26, ControllerNeutralMode::Brake, false, "rio" };
	OverTalonFX lowerShooterMotor{ 27, ControllerNeutralMode::Brake, false, "rio" };

	const double LOWER_GEAR_BOX_REDUCTION = 1.0/2.4;
	const double UPPER_GEAR_BOX_REDUCTION = 1.0/2.4;

	double velocity;


	frc2::sysid::SysIdRoutine sysIdRoutine{
	frc2::sysid::Config{0.5_V / 1_s, 3_V, 6_s,
						std::nullopt},
	frc2::sysid::Mechanism{
		[this](units::volt_t driveVoltage) {
			upperShooterMotor.SetVoltage(driveVoltage);
			lowerShooterMotor.SetVoltage(driveVoltage);
		},
		[this](frc::sysid::SysIdRoutineLog* log) {

		log->Motor("ShooterUpper")
			.voltage(upperShooterMotor.GetMotorVoltage().GetValue())
			.position(upperShooterMotor.GetPosition().GetValue())
			.velocity(upperShooterMotor.GetVelocity().GetValue());

		log->Motor("ShooterLower")
			.voltage(lowerShooterMotor.GetMotorVoltage().GetValue())
			.position(lowerShooterMotor.GetPosition().GetValue())
			.velocity(lowerShooterMotor.GetVelocity().GetValue());

		},
		this} };
};
