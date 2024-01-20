// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
#ifndef __FRC_ROBORIO__
  simMotorManager->Init({
    {1, "7421-24/motors/SDS_Module_FL_rotation_joint"}, 
    {2, "7421-24/motors/SDS_Module_FL_wheel_joint"},
    
    {3, "7421-24/motors/SDS_Module_FR_rotation_joint"}, 
    {4, "7421-24/motors/SDS_Module_FR_wheel_joint"},

    {5, "7421-24/motors/SDS_Module_BR_rotation_joint"}, 
    {6, "7421-24/motors/SDS_Module_BR_wheel_joint"},

    {7, "7421-24/motors/SDS_Module_BL_rotation_joint"}, 
    {8, "7421-24/motors/SDS_Module_BL_wheel_joint"},

    {20, "7421-24/motors/chassis_arm_joint_right"},
    {21, "7421-24/motors/chassis_arm_joint_left"},
    {22, "7421-24/motors/arm_shooter_joint"},
    {23, "7421-24/motors/chassis_supports_joint"},

    {24, "7421-24/motors/shooter_storage_wheel_left_joint"},
    {25, "7421-24/motors/chassis_intake_wheels_joint"},
    {26, "7421-24/motors/shooter_roller_up_joint_up"}
    {27, "7421-24/motors/shooter_roller_up_joint_down"}

    });

  simPigeonManager->Init("7421-24/chassis/imu_sensor");

  simCANCoderManager->Init({
    {9,  "7421-24/cancoders/SDS_Module_FL_rotation_joint"}, 
    {10, "7421-24/cancoders/SDS_Module_FR_rotation_joint"},
    {11, "7421-24/cancoders/SDS_Module_BR_rotation_joint"},
    {12, "7421-24/cancoders/SDS_Module_BL_rotation_joint"},
    });

  simDutyCycleEncoderManager->Init({
    {0, "7421-24/cancoders/chassis_arm_joint"}, 
    {1, "7421-24/cancoders/arm_shooter_joint"},
    {2, "7421-24/cancoders/chassis_supports_joint"},
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

