// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
#ifndef __FRC_ROBORIO__
  simMotorManager->Init({
    {1, "FirstCAD7421/motors/SDS_Module_FL_rotation_joint"}, 
    {2, "FirstCAD7421/motors/SDS_Module_FL_wheel_joint"},
    
    {3, "FirstCAD7421/motors/SDS_Module_FR_rotation_joint"}, 
    {4, "FirstCAD7421/motors/SDS_Module_FR_wheel_joint"},

    {5, "FirstCAD7421/motors/SDS_Module_BR_rotation_joint"}, 
    {6, "FirstCAD7421/motors/SDS_Module_BR_wheel_joint"},

    {7, "FirstCAD7421/motors/SDS_Module_BL_rotation_joint"}, 
    {8, "FirstCAD7421/motors/SDS_Module_BL_wheel_joint"},

    {20, "FirstCAD7421/motors/chassis_arm_joint_right"},
    {21, "FirstCAD7421/motors/chassis_arm_joint_left"},
    {22, "FirstCAD7421/motors/arm_shooter_joint"},
    {23, "FirstCAD7421/motors/chassis_supports_joint"},

    {24, "FirstCAD7421/motors/shooter_storage_wheel_left_joint"},
    {25, "FirstCAD7421/motors/chassis_intake_wheels_joint"},
    {26, "FirstCAD7421/motors/shooter_roller_up_joint"},
    {27, "FirstCAD7421/motors/shooter_roller_down_joint"}
    });

  simPigeonManager->Init("FirstCAD7421/chassis/imu_sensor");

  simCANCoderManager->Init({
    {9,  "FirstCAD7421/cancoders/SDS_Module_FL_rotation_joint"}, 
    {10, "FirstCAD7421/cancoders/SDS_Module_FR_rotation_joint"},
    {11, "FirstCAD7421/cancoders/SDS_Module_BR_rotation_joint"},
    {12, "FirstCAD7421/cancoders/SDS_Module_BL_rotation_joint"},
    });

  simDutyCycleEncoderManager->Init({
    {0, "FirstCAD7421/cancoders/chassis_arm_joint"}, 
    {1, "FirstCAD7421/cancoders/arm_shooter_joint"},
    {2, "FirstCAD7421/cancoders/chassis_supports_joint"},
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

