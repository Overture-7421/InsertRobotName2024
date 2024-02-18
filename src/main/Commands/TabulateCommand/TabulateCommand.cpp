// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TabulateCommand.h"

TabulateCommand::TabulateCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  this->chassis = chassis;

  AddRequirements({superStructure, shooter});
}

// Called when the command is initially scheduled.
void TabulateCommand::Initialize() {
  frc::SmartDashboard::PutNumber("Tabulate/LowerAngle", superStructure->getLowerAngle());
  frc::SmartDashboard::PutNumber("Tabulate/UpperAngle", superStructure->getUpperAngle());
  frc::SmartDashboard::PutNumber("Tabulate/ShooterVel", 0.0);

  if(shouldFlip()){
    targetLocation = pathplanner::GeometryUtil::flipFieldPosition({0.06_m, 5.54_m});
  }else{
    targetLocation = {0.06_m, 5.54_m};
  }

  chassis->setHeadingOverride(true);
}

// Called repeatedly when this Command is scheduled to run
void TabulateCommand::Execute() {

  frc::Pose2d chassisPose = chassis->getOdometry();
  frc::Translation2d chassisLoc = chassisPose.Translation();

  frc::Translation2d chassisToTarget = targetLocation - chassisLoc;
  auto distance = chassisToTarget.Distance({0_m, 0_m});
  auto angle = chassisToTarget.Angle().RotateBy({-180_deg});

  frc::SmartDashboard::PutNumber("Tabulate/Distance", distance.value());
  frc::SmartDashboard::PutNumber("Tabulate/LowerAngleCurrent", superStructure->getLowerAngle());
  frc::SmartDashboard::PutNumber("Tabulate/UpperAngleCurrent", superStructure->getUpperAngle());
  frc::SmartDashboard::PutNumber("Tabulate/ShooterVelCurrent", shooter->getCurrentVelocity());

  chassis->setTargetHeading(angle);
  double lowerAngle = frc::SmartDashboard::GetNumber("Tabulate/LowerAngle", superStructure->getLowerAngle());
  double upperAngle = frc::SmartDashboard::GetNumber("Tabulate/UpperAngle", superStructure->getLowerAngle());
  double targetVel = frc::SmartDashboard::GetNumber("Tabulate/ShooterVel", 0.0);

  superStructure->setTargetCoord({lowerAngle, upperAngle});
  shooter->setVelocityVoltage(targetVel);
}

// Called once the command ends or is interrupted.
void TabulateCommand::End(bool interrupted) {
    shooter->setVelocityVoltage(0.0);
    chassis->setHeadingOverride(false);
}

// Returns true when the command should end.
bool TabulateCommand::IsFinished() {
  return false;
}
