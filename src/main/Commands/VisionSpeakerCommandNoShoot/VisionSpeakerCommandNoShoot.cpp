// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommandNoShoot.h"

#include <frc/MathUtil.h>

VisionSpeakerCommandNoShoot::VisionSpeakerCommandNoShoot(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, AprilTagCamera* tagCamera) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->tagCamera = tagCamera;
}

// Called when the command is initially scheduled.
void VisionSpeakerCommandNoShoot::Initialize() {

	targetLocation = tagCamera->GetSpeakerLocation();

	chassis->setHeadingOverride(true);
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommandNoShoot::Execute() {
	frc::Pose2d chassisPose = chassis->getOdometry();
	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = targetLocation - chassisLoc;
	distance = chassisToTarget.Distance({ 0_m, 0_m });
	angle = chassisToTarget.Angle().RotateBy({ -180_deg });


	chassis->setTargetHeading(angle);
	double targetLowerAngle = VisionSpeakerCommandConstants::DistanceToLowerAngleTable[distance];
	double targetUpperAngle = VisionSpeakerCommandConstants::DistanceToUpperAngleTable[distance];
	double targetShooterVelocity = VisionSpeakerCommandConstants::DistanceToVelocityTable[distance];
	superStructure->setTargetCoord({ targetLowerAngle, targetUpperAngle });
	shooter->setTargetVelocity(targetShooterVelocity);
}

// Called once the command ends or is interrupted.
void VisionSpeakerCommandNoShoot::End(bool interrupted) {
	chassis->setHeadingOverride(false);
}

// Returns true when the command should end.
bool VisionSpeakerCommandNoShoot::IsFinished() {
	return false;
}