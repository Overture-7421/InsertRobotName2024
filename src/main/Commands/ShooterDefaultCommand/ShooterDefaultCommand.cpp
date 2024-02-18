// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShooterDefaultCommand.h"

ShooterDefaultCommand::ShooterDefaultCommand(Chassis* chassis, Shooter* shooter) {
  this->shooter = shooter;
  this->chassis = chassis;
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void ShooterDefaultCommand::Initialize() {
  if (shouldFlip()) {
		targetLocation = pathplanner::GeometryUtil::flipFieldPosition(VisionSpeakerCommandConstants::TargetLocation);
	} else {
		targetLocation = VisionSpeakerCommandConstants::TargetLocation;
	}
}

// Called repeatedly when this Command is scheduled to run
void ShooterDefaultCommand::Execute() {
  frc::Pose2d chassisPose = chassis->getOdometry();
  frc::Translation2d chassisLoc = chassisPose.Translation();

  frc::Translation2d chassisToTarget = targetLocation - chassisLoc;
  auto distance = chassisToTarget.Distance({ 0_m, 0_m });

  double targetShooterVelocity = VisionSpeakerCommandConstants::DistanceToVelocityTable[distance];
  shooter->setVelocityVoltage(targetShooterVelocity);
}

// Called once the command ends or is interrupted.
void ShooterDefaultCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ShooterDefaultCommand::IsFinished() {
  return false;
}
