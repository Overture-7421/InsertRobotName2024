// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angular_acceleration.h>

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
	SupportArmsState getCurrentState();
	void Periodic() override;

	// frc2::CommandPtr sysIdQuadstaticLower(frc2::sysid::Direction direction) {
	// 	return sysIdRoutineLower.Quasistatic(direction);
	// }

	// frc2::CommandPtr sysIdDinamicLower(frc2::sysid::Direction direction) {
	// 	return sysIdRoutineLower.Dynamic(direction);
	// }

private:
	void setFalconTargetPos(SupportArmsState targetState, SupportArmsState currentState);
	double convertAngleToFalconPos(double angle);

	//constant
	const double LOWER_GEAR_BOX_REDUCTION = 80.1818;

	//Encoders
	OverDutyCycleEncoder lowerEncoder{ 6 };

	#ifdef __FRC_ROBORIO__
	double lowerOffset = -0.761712;
	#else
	double lowerOffset = 0.0;
	#endif

	// LowerMotors
	// OverTalonFX lowerRightMotor{ 23, ControllerNeutralMode::Brake, true, "rio" };

	// State
	SupportArmsState m_TargetState{ getCurrentState() };
	SupportArmsPosition position = SupportArmsPosition::Closed;

	// //Feed Forward
	// frc::ArmFeedforward lowerFF {0.32417_V, 0.068615_V, 7.975_V / 1_tps, 0.081073_V / 1_tr_per_s_sq }; 

	// frc2::sysid::SysIdRoutine sysIdRoutineLower{
	// 	frc2::sysid::Config{0.75_V / 1_s, 4_V, 3_s,
	// 						std::nullopt},
	// 	frc2::sysid::Mechanism{
	// 		[this](units::volt_t driveVoltage) {
	// 			lowerRightMotor.SetVoltage(driveVoltage);
	// 		},
	// 		[this](frc::sysid::SysIdRoutineLog* log) {

	// 		log->Motor("SupportArmsLower")
	// 			.voltage(lowerRightMotor.GetMotorVoltage().GetValue())
	// 			.position(lowerRightMotor.GetPosition().GetValue())
	// 			.velocity(lowerRightMotor.GetVelocity().GetValue());
	// 		},
	// 	this} };
};
