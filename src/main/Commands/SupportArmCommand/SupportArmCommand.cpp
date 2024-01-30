// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArmCommand.h"

SupportArmCommand2::SupportArmCommand2(SupportArms* supportarms, SupportArmsState targetstate) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->m_supportarms = supportarms;
  this->m_targetState = targetstate;
  AddRequirements(m_supportarms);
}

// Called when the command is initially scheduled.
void SupportArmCommand2::Initialize() {
  	m_supportarms->setTargetCoord(m_targetState);
}

// Called repeatedly when this Command is scheduled to run
void SupportArmCommand2::Execute() {}

// Called once the command ends or is interrupted.
void SupportArmCommand2::End(bool interrupted) {}

// Returns true when the command should end.
bool SupportArmCommand2::IsFinished() {
  if (m_supportarms->getLowerAngle() == m_targetState.lowerAngle) {
	return true;
	}
	else{
  return false;
  }
}
