// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShooterDefaultCommand.h"

ShooterDefaultCommand::ShooterDefaultCommand(Chassis* chassis, Shooter* shooter, const frc::AprilTagFieldLayout* layout) {
  this->shooter = shooter;
  this->chassis = chassis;
  this->layout = layout;
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void ShooterDefaultCommand::Initialize() {
	if (isRedAlliance()) {
		targetLocation = layout->GetTagPose(4).value().ToPose2d().Translation();
	} else {
		targetLocation = layout->GetTagPose(7).value().ToPose2d().Translation();
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
