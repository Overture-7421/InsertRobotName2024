// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommandNoShoot.h"
#include "Commands/VisionSpeakerCommand/VisionSpeakerCommand.h"
#include <frc/MathUtil.h>

VisionSpeakerCommandNoShoot::VisionSpeakerCommandNoShoot(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, TargetProvider* targetProvider) : headingHelper({ 11.0, 0.5, 0.6, {13_rad_per_s, 18_rad_per_s_sq * 2} }, chassis) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->targetProvider = targetProvider;
}

// Called when the command is initially scheduled.
void VisionSpeakerCommandNoShoot::Initialize() {

	targetLocation = targetProvider->GetSpeakerLocation();

	chassis->enableSpeedHelper(&headingHelper);
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommandNoShoot::Execute() {
	frc::Pose2d chassisPose = chassis->getEstimatedPose();
	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = targetLocation - chassisLoc;
	distance = chassisToTarget.Distance({ 0_m, 0_m });
	angle = chassisToTarget.Angle().RotateBy({ -180_deg });


	headingHelper.setTargetAngle(angle.Radians());
	double targetLowerAngle = VisionSpeakerCommandConstants::DistanceToLowerAngleTable[distance];
	double targetUpperAngle = VisionSpeakerCommandConstants::DistanceToUpperAngleTable[distance] + VisionSpeakerCommand::GetUpperAngleOffset();
	double targetShooterVelocity = VisionSpeakerCommandConstants::DistanceToVelocityTable[distance];
	superStructure->setTargetCoord({ targetLowerAngle, targetUpperAngle });
	shooter->setTargetVelocity(targetShooterVelocity);
}

// Called once the command ends or is interrupted.
void VisionSpeakerCommandNoShoot::End(bool interrupted) {
	chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool VisionSpeakerCommandNoShoot::IsFinished() {
	return false;
}