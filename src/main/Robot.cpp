// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
#ifndef __FRC_ROBORIO__
  simMotorManager->Init({
    {1, "SDS_Chassis/motors/SDS_Module_FL_rotation_joint"}, 
    {2, "SDS_Chassis/motors/SDS_Module_FL_wheel_joint"},
    
    {3, "SDS_Chassis/motors/SDS_Module_FR_rotation_joint"}, 
    {4, "SDS_Chassis/motors/SDS_Module_FR_wheel_joint"},

    {5, "SDS_Chassis/motors/SDS_Module_BR_rotation_joint"}, 
    {6, "SDS_Chassis/motors/SDS_Module_BR_wheel_joint"},

    {7, "SDS_Chassis/motors/SDS_Module_BL_rotation_joint"}, 
    {8, "SDS_Chassis/motors/SDS_Module_BL_wheel_joint"}

    });

  simPigeonManager->Init("SDS_Chassis/Frames/imu_sensor");

  simCANCoderManager->Init({
    {9,  "SDS_Chassis/cancoders/SDS_Module_FL_rotation_joint"}, 
    {10, "SDS_Chassis/cancoders/SDS_Module_FR_rotation_joint"},
    {11, "SDS_Chassis/cancoders/SDS_Module_BR_rotation_joint"},
    {12, "SDS_Chassis/cancoders/SDS_Module_BL_rotation_joint"},
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

