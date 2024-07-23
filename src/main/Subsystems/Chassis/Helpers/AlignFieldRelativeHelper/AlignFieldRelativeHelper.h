// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include "Commands/AlignToTrackedObject/Constants.h"

class AlignFieldRelativeHelper : public SpeedsHelper {
public:
	AlignFieldRelativeHelper(SwerveChassis* chassis);
	void setTargetPosition(units::meter_t xPosition, units::meter_t yPosition);
	void alterSpeed(frc::ChassisSpeeds& inputSpeed) override;

private:
	frc::ProfiledPIDController<units::meter> xController{ 0.1, 0.0, 0.0, {2_mps, 5_mps_sq}, RobotConstants::LoopTime };
	frc::ProfiledPIDController<units::meter> yController{ 0.1, 0.0, 0.0, {2_mps, 5_mps_sq}, RobotConstants::LoopTime };
	SwerveChassis* m_chassis = nullptr;
	units::meter_t m_xPosition;
	units::meter_t m_yPosition;
};