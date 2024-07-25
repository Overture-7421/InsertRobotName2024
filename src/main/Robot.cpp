// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/DataLogManager.h>

void Robot::RobotInit() {
	// m_teleopResetCommand = m_container.GetTeleopResetCommand();

	AddPeriodic([&] {
		frc2::CommandScheduler::GetInstance().Run();
	}, RobotConstants::LoopTime, RobotConstants::TimingOffset);

#ifndef __FRC_ROBORIO__
	simMotorManager->Init({
	  {1, "Sample Robot/motors/back_right"},
	  {3, "Sample Robot/motors/back_left"},
	  {5, "Sample Robot/motors/front_left"},
	  {7, "Sample Robot/motors/front_right"},

	  {2, "Sample Robot/motors/back_right_drive"},
	  {4, "Sample Robot/motors/back_left_drive"},
	  {6, "Sample Robot/motors/front_left_drive"},
	  {8, "Sample Robot/motors/front_right_drive"},

	});

	simPigeonManager->Init("Vantage7421/chassis/imu_sensor");

	simCANCoderManager->Init({
	  {9, "Sample Robot/cancoders/back_right"},
	  {10, "Sample Robot/cancoders/back_left"},
	  {11, "Sample Robot/cancoders/front_left"},
	  {12, "Sample Robot/cancoders/front_right"}
	});

	simDutyCycleEncoderManager->Init({

		});

#endif

	photon::PhotonCamera::SetVersionCheckEnabled(false);
	frc::DriverStation::SilenceJoystickConnectionWarning(true);
}

void Robot::RobotPeriodic() {
	// m_container.UpdateTelemetry();
}

void Robot::DisabledInit() {
	frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
	// m_autonomousCommand = m_container.GetAutonomousCommand();

	// if (m_autonomousCommand) {
	// 	m_autonomousCommand->Schedule();
	// }

	// VisionSpeakerCommand::LoadAllianceOffset();
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
	// if (m_autonomousCommand) {
	// 	m_autonomousCommand->Cancel();
	// }
	// m_teleopResetCommand->Schedule();
}

void Robot::TeleopPeriodic() {
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
	frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif

