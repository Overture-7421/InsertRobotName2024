// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructureCommand.h"

SuperStructureCommand::SuperStructureCommand(SuperStructure* superStructure, SuperStructureState targetState) {
	this->m_SuperStructure = superStructure;
	this->m_targetState = targetState;
	AddRequirements(m_SuperStructure);
}

// Called when the command is initially scheduled.
void SuperStructureCommand::Initialize() {
	m_SuperStructure->setTargetCoord(m_targetState);
}

// Called repeatedly when this Command is scheduled to run
void SuperStructureCommand::Execute() {}

// Called once the command ends or is interrupted.
void SuperStructureCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool SuperStructureCommand::IsFinished() {
	double lowError = abs(m_targetState.lowerAngle - m_SuperStructure->getLowerAngle());
	double upperError = abs(m_targetState.upperAngle - m_SuperStructure->getUpperAngle());

	if (lowError <= 2 && upperError <= 2){
	return true;
	}
	else {
		return false;
	}
}