// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Robots/OverRobot/RobotConstants.h>
#include <units/angular_acceleration.h>

class AlignRobotRelativeHelper : public SpeedsHelper {
public:
	AlignRobotRelativeHelper();
	void setCurrentAngle(units::degree_t position);
	void alterSpeed(frc::ChassisSpeeds& inputSpeed) override;
	bool isNoteDetected();
	void setNoteDetected(units::degree_t initialPosition);
	void setNoteLost();
private:
	frc::ProfiledPIDController<units::degree> controller{ 15, 0.0, 0.0, {3_deg_per_s, 7_deg_per_s_sq}, RobotConstants::LoopTime };
	units::degree_t m_currentAngle;
	bool noteDetected = false;
};
