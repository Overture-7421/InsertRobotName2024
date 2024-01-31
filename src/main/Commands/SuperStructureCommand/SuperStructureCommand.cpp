// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructureCommand.h"

SuperStructureCommand::SuperStructureCommand(SuperStructure* superStructure, SuperStructureState targetState) {
	this->superStructure = superStructure;
	this->targetState = targetState;
	AddRequirements(superStructure);
}

// Called when the command is initially scheduled.
void SuperStructureCommand::Initialize() {
	superStructure->setTargetCoord(targetState);
}

// Called repeatedly when this Command is scheduled to run
void SuperStructureCommand::Execute() {}

// Called once the command ends or is interrupted.
void SuperStructureCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool SuperStructureCommand::IsFinished() {
	double lowError = abs(targetState.lowerAngle - superStructure->getLowerAngle());
	double upperError = abs(targetState.upperAngle - superStructure->getUpperAngle());

	if (lowError <= 2 && upperError <= 2){
	return true;
	}
	else {
		return false;
	}
}