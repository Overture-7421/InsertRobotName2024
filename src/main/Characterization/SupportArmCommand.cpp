// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArmCommand.h"

SupportArmCommand::SupportArmCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SupportArmCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SupportArmCommand::Execute() {}

// Called once the command ends or is interrupted.
void SupportArmCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool SupportArmCommand::IsFinished() {
  return false;
}
