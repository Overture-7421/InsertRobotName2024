// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommand.h"

VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter});
  this->chassis = chassis;
  this->superStructure = superStructure;
  this->shooter = shooter;
}

// Called when the command is initially scheduled.
void VisionSpeakerCommand::Initialize() {
  chassis->setHeadingOverride(true);
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommand::Execute() {
  frc::Pose2d chassisPose = chassis->getOdometry();
  frc::Translation2d speakerLoc = dynamicTarget.getMovingTarget(chassisPose, chassis->getFieldRelativeSpeeds(), chassis->getFIeldRelativeAccels());
  frc::Translation2d chassisLoc = chassisPose.Translation();

  frc::Translation2d chassisToTarget = speakerLoc - chassisLoc;
  distance = chassisToTarget.Distance({0_m, 0_m});
  angle = chassisToTarget.Angle().RotateBy({180_deg});
  frc::SmartDashboard::PutNumber("Distance to Target", double(distance));
  frc::SmartDashboard::PutNumber("Angle to Target", angle.Degrees().value());

  chassis->setTargetHeading(angle);
  superStructure->setTargetCoord({distanceToLowerAngleTable[distance], distanceToUpperAngleTable[distance]});
  shooter->setVelocityVoltage(distanceToVelocityTable[distance]);

  
}

// Called once the command ends or is interrupted.
void VisionSpeakerCommand::End(bool interrupted) {
  chassis->setHeadingOverride(false);
}

// Returns true when the command should end.
bool VisionSpeakerCommand::IsFinished() {
  return false;
}
