// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ResetAngle.h"

ResetAngle::ResetAngle(SwerveChassis* swerveChassis) : swerveChassis(swerveChassis) {
	// Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ResetAngle::Initialize() {
	frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? angle = 180 : angle = 0;
}

// Called repeatedly when this Command is scheduled to run
void ResetAngle::Execute() {
	swerveChassis->resetAngle(angle);
}

// Called once the command ends or is interrupted.
void ResetAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool ResetAngle::IsFinished() {
	return true;
}
