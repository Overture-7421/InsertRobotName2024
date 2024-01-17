// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ResetAngle.h"

ResetAngle::ResetAngle(SwerveChassis* swerveChassis) : m_swerveChassis(swerveChassis) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ m_swerveChassis });
}

// Called when the command is initially scheduled.
void ResetAngle::Initialize() {
	frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? m_angle = 180 : m_angle = 0;
}

// Called repeatedly when this Command is scheduled to run
void ResetAngle::Execute() {
	m_swerveChassis->resetAngle(m_angle);
}

// Called once the command ends or is interrupted.
void ResetAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool ResetAngle::IsFinished() {
	return false;
}
