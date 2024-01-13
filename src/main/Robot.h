// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc2/command/CommandPtr.h>

#include "OvertureLib/Robots/OverRobot/OverRobot.h"
#include "Characterization/SysIDRoutineBot.h"

#include "RobotContainer.h"

class Robot : public OverRobot {
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void DisabledExit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void AutonomousExit() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TeleopExit() override;
	void TestInit() override;
	void TestPeriodic() override;
	void TestExit() override;

private:
	std::optional<frc2::CommandPtr> m_autonomousCommand;

	OverTalonFX armMotor {0, ControllerNeutralMode::Brake, false, ""};
	// RobotContainer m_container;
	// SysIDRoutineBot m_container;
};
