// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
#ifndef __FRC_ROBORIO__
  simMotorManager->Init({
    {1, "FirstRobotConcept/motors/SDS_Module_FL_rotation_joint"}, 
    {2, "FirstRobotConcept/motors/SDS_Module_FL_wheel_joint"},
    
    {3, "FirstRobotConcept/motors/SDS_Module_FR_rotation_joint"}, 
    {4, "FirstRobotConcept/motors/SDS_Module_FR_wheel_joint"},

    {5, "FirstRobotConcept/motors/SDS_Module_BR_rotation_joint"}, 
    {6, "FirstRobotConcept/motors/SDS_Module_BR_wheel_joint"},

    {7, "FirstRobotConcept/motors/SDS_Module_BL_rotation_joint"}, 
    {8, "FirstRobotConcept/motors/SDS_Module_BL_wheel_joint"},

    {20, "FirstRobotConcept/motors/arm_joint"},
    {22, "FirstRobotConcept/motors/shooter_arm_joint"},
    {23, "FirstRobotConcept/motors/supports_joint"},
    {25, "FirstRobotConcept/motors/intake_wheels_joint"},
    {26, "FirstRobotConcept/motors/shooter_roller_joint"}

    });

  simPigeonManager->Init("FirstRobotConcept/chassis/imu_sensor");

  simCANCoderManager->Init({
    {9,  "FirstRobotConcept/cancoders/SDS_Module_FL_rotation_joint"}, 
    {10, "FirstRobotConcept/cancoders/SDS_Module_FR_rotation_joint"},
    {11, "FirstRobotConcept/cancoders/SDS_Module_BR_rotation_joint"},
    {12, "FirstRobotConcept/cancoders/SDS_Module_BL_rotation_joint"},
    });

  simDutyCycleEncoderManager->Init({
    {0, "FirstRobotConcept/cancoders/arm_joint"}, 
    {1, "FirstRobotConcept/cancoders/shooter_arm_joint"},
    {2, "FirstRobotConcept/cancoders/supports_joint"},
  });

#endif
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

void Robot::DisabledExit() {
}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
     m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
}

void Robot::AutonomousExit() {
}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
}

void Robot::TeleopExit() {
}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {
}

void Robot::TestExit() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

