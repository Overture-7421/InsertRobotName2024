// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommand.h"

#include <frc/MathUtil.h>

VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, frc::XboxController* joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter});
  this->chassis = chassis;
  this->superStructure = superStructure;
  this->shooter = shooter;
  this->joystick = joystick;
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
  double targetLowerAngle = distanceToLowerAngleTable[distance];
  double targetUpperAngle = distanceToUpperAngleTable[distance];
  double targetShooterVelocity = distanceToVelocityTable[distance];
  superStructure->setTargetCoord({targetLowerAngle, targetUpperAngle});
  shooter->setVelocityVoltage(targetShooterVelocity);

  bool lowerAngleInTolerance = std::abs(targetLowerAngle - superStructure->getLowerAngle()) < 1.00;
  bool upperAngleInTolerance = std::abs(targetUpperAngle - superStructure->getUpperAngle()) < 1.00;
  bool headingInTolerance = units::math::abs(frc::InputModulus(angle.Degrees() - chassisPose.Rotation().Degrees(), -180_deg, 180_deg)) < 3_deg; 
  bool shooterSpeedInTolerance = std::abs(targetShooterVelocity - shooter->getCurrentVelocity()) < 5.00; 

  frc::SmartDashboard::PutBoolean("lowerAngleInTolerance ", lowerAngleInTolerance);
  frc::SmartDashboard::PutBoolean("upperAngleInTolerance ", upperAngleInTolerance);
  frc::SmartDashboard::PutBoolean("headingInTolerance ", headingInTolerance);
  frc::SmartDashboard::PutBoolean("shooterSpeedInTolerance ", shooterSpeedInTolerance);
  

  if (lowerAngleInTolerance && upperAngleInTolerance && headingInTolerance && shooterSpeedInTolerance) {
    joystick->SetRumble(frc::GenericHID::kBothRumble, 1.0);
  } else {
    joystick->SetRumble(frc::GenericHID::kBothRumble, 0.0);
  }

}

// Called once the command ends or is interrupted.
void VisionSpeakerCommand::End(bool interrupted) {
  chassis->setHeadingOverride(false);
  joystick->SetRumble(frc::GenericHID::kBothRumble, 0.0);
}

// Returns true when the command should end.
bool VisionSpeakerCommand::IsFinished() {
  return false;
}
