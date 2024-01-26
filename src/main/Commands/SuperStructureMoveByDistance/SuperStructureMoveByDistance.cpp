// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructureMoveByDistance.h"

SuperStructureMoveByDistance::SuperStructureMoveByDistance(SuperStructure* superStructure, Profile profile, std::function<units::meter_t()> distanceToTargetProvider) {
	this->superStructure = superStructure;
	this->profile = profile;
	this->distanceToTargetProvider = distanceToTargetProvider;

	upperAngleTravel = profile.targetState.upperAngle - profile.startingState.upperAngle;
	lowerAngleTravel = profile.targetState.lowerAngle - profile.startingState.lowerAngle;
	AddRequirements(superStructure);
}

// Called when the command is initially scheduled.
void SuperStructureMoveByDistance::Initialize() {
	superStructure->setTargetCoord(profile.startingState);
}

// Called repeatedly when this Command is scheduled to run
void SuperStructureMoveByDistance::Execute() {
	distanceToTarget = distanceToTargetProvider();
	frc::SmartDashboard::PutNumber("DistanceMoveByDistance", distanceToTarget.value());
	SuperStructureState targetState = profile.targetState;

	if (distanceToTarget < profile.profileActivationDistance) {
		double inverseNormalizedDistance = (distanceToTarget / profile.profileActivationDistance).value();
		targetState.upperAngle = profile.startingState.upperAngle + upperAngleTravel * inverseNormalizedDistance;
		targetState.lowerAngle = profile.startingState.lowerAngle + lowerAngleTravel * inverseNormalizedDistance;
	}

	superStructure->setTargetCoord(targetState);
}

// Called once the command ends or is interrupted.
void SuperStructureMoveByDistance::End(bool interrupted) {}

// Returns true when the command should end.
bool SuperStructureMoveByDistance::IsFinished() {
	return distanceToTarget < 0.01_m;
}
