// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>

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
	void setFalconTargetPos(SupportArmsState targetState, SupportArmsState currentState);
	double convertAngleToFalconPos(double angle);

	//constant
	const double LOWER_GEAR_BOX_REDUCTION = 106.0;

	//Encoders
	OverDutyCycleEncoder lowerEncoder{ 2 };
	double lowerOffset = 0;


	// LowerMotors
	// OverTalonFX m_lowerRight{ 23, ControllerNeutralMode::Brake, true, "rio" };

 	 rev::CANSparkMax m_lowerRight {23, rev::CANSparkMax::MotorType::kBrushless};
	 rev::SparkPIDController pidController = m_lowerRight.GetPIDController();
	 rev::SparkRelativeEncoder encoder = m_lowerRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	 frc::TrapezoidProfile<units::degree_t> trapezoidProfile {{0_deg_per_s, 0_deg_per_s_sq}};
	 frc::TrapezoidProfile<units::degree_t>::State setpoint;
	 units::second_t dt = 20_ms;

	// State
	SupportArmsState m_TargetState{ getCurrentState() };
	SupportArmsPosition position = SupportArmsPosition::Closed;

	//Motion Magic Feed Forward
	double lowerFF = 0.0;
};
