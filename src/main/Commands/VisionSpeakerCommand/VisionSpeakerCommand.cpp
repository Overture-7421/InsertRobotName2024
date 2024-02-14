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

  if(shouldFlip()){
    dynamicTarget.setTargetLocation(pathplanner::GeometryUtil::flipFieldPosition({0.06_m, 5.54_m}));
  }else{
    dynamicTarget.setTargetLocation({0.06_m, 5.54_m});
  }
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommand::Execute() {
  frc::Pose2d chassisPose = chassis->getOdometry();
  frc::Translation2d speakerLoc = dynamicTarget.getMovingTarget(chassisPose, chassis->getFieldRelativeSpeeds(), chassis->getFIeldRelativeAccels());
  frc::Translation2d chassisLoc = chassisPose.Translation();

  frc::Translation2d chassisToTarget = speakerLoc - chassisLoc;
  distance = chassisToTarget.Distance({0_m, 0_m});
  angle = chassisToTarget.Angle().RotateBy({180_deg});

  chassis->setTargetHeading(angle);
  double targetLowerAngle = distanceToLowerAngleTable[distance];
  double targetUpperAngle = distanceToUpperAngleTable[distance];
  double targetShooterVelocity = distanceToVelocityTable[distance];
  superStructure->setTargetCoord({targetLowerAngle, targetUpperAngle});
  shooter->setVelocityVoltage(targetShooterVelocity);

	units::degree_t headingTolerance = 2_deg + units::degree_t(std::clamp(1 - distance.value() / 8.0, 0.0, 1.0) * 5); // Heading tolerance extra of X deg when close, more precise when further back;
	units::degree_t headingError = units::math::abs(frc::InputModulus(angle.Degrees() - chassisPose.Rotation().Degrees(), -180_deg, 180_deg));


  bool lowerAngleInTolerance = std::abs(targetLowerAngle - superStructure->getLowerAngle()) < 1.00;
  bool upperAngleInTolerance = std::abs(targetUpperAngle - superStructure->getUpperAngle()) < 1.00;
  bool headingInTolerance = headingError < headingTolerance; 
  bool shooterSpeedInTolerance = std::abs(targetShooterVelocity - shooter->getCurrentVelocity()) < 5.00; 

  frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/LowerAngleReached", lowerAngleInTolerance);
  frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/UpperAngleReached", upperAngleInTolerance);
  frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/HeadingReached", headingInTolerance);
  frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/ShooterReached", shooterSpeedInTolerance);


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
