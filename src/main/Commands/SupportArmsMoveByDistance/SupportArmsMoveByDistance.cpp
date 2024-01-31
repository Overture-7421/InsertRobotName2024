// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArmsMoveByDistance.h"

SupportArmsMoveByDistance::SupportArmsMoveByDistance(SupportArms* supportArms, Profile profile, std::function<units::meter_t()> distanceToTargetProvider) {
	this->supportArms = supportArms;
	this->profile = profile;
	this->distanceToTargetProvider = distanceToTargetProvider;

	lowerAngleTravel = profile.targetState.lowerAngle - profile.startingState.lowerAngle;
	AddRequirements(supportArms);
}

// Called when the command is initially scheduled.
void SupportArmsMoveByDistance::Initialize() {
	supportArms->setTargetCoord(profile.startingState);
}

// Called repeatedly when this Command is scheduled to run
void SupportArmsMoveByDistance::Execute() {
	distanceToTarget = units::math::abs(distanceToTargetProvider());
	frc::SmartDashboard::PutNumber("SupportArmsMoveByDistance/DistanceMoveByDistance", distanceToTarget.value());

	if (distanceToTarget < profile.profileActivationDistance) {
		SupportArmsState targetState;

		double inverseNormalizedDistance = 1.0 - (distanceToTarget / profile.profileActivationDistance).value();
		frc::SmartDashboard::PutNumber("SupportArmsMoveByDistance/NormalizedDistance", inverseNormalizedDistance);
		targetState.lowerAngle = profile.startingState.lowerAngle + lowerAngleTravel * inverseNormalizedDistance;

		supportArms->setTargetCoord(targetState);
	}

}

// Called once the command ends or is interrupted.
void SupportArmsMoveByDistance::End(bool interrupted) {}

// Returns true when the command should end.
bool SupportArmsMoveByDistance::IsFinished() {
	return distanceToTarget < 0.01_m;
}
