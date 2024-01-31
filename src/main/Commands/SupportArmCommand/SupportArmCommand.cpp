// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArmCommand.h"

SupportArmCommand::SupportArmCommand(SupportArms* supportarms, SupportArmsState targetstate) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->m_supportarms = supportarms;
  this->m_targetState = targetstate;
  AddRequirements(m_supportarms);
}

// Called when the command is initially scheduled.
void SupportArmCommand::Initialize() {
  	m_supportarms->setTargetCoord(m_targetState);
}

// Called repeatedly when this Command is scheduled to run
void SupportArmCommand::Execute() {}

// Called once the command ends or is interrupted.
void SupportArmCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool SupportArmCommand::IsFinished() {
  double error = abs(m_targetState.lowerAngle - m_supportarms->getLowerAngle());

  if (error <= 2 ) {
	return true;
	}
	else{
  return false;
  }
}
