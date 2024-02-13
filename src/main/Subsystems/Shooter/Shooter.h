// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
public:
	Shooter();
	void setVelocityVoltage(double velocity);
	double getCurrentVelocity();
	void setVoltage(units::volt_t voltage);
	void Periodic() override;

	frc2::CommandPtr sysIdQuasistatic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Quasistatic(direction);
	}

	frc2::CommandPtr sysIdDynamic(frc2::sysid::Direction direction) {
		return sysIdRoutine.Dynamic(direction);
	}

private:
	double getUpperMotorCurrentVelocity();
	double getLowerMotorCurrentVelocity();
	
	OverTalonFX upperShooterMotor{ 26, ControllerNeutralMode::Brake, false, "rio" };
	OverTalonFX lowerShooterMotor{ 27, ControllerNeutralMode::Brake, false, "rio" };

	frc::SimpleMotorFeedforward<units::turn> upperFF {0.23595_V, 0.049669_V / 1_tps, 0.0057865_V / 1_tr_per_s_sq};
	frc::SimpleMotorFeedforward<units::turn> lowerFF {0.26217_V, 0.04915_V / 1_tps, 0.0062472_V / 1_tr_per_s_sq};

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
