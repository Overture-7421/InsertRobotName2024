// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/Commands.h>

static frc2::CommandPtr WaitForAButton(frc::XboxController* controller, int buttonNumber) {
	return frc2::cmd::WaitUntil([=]() {return controller->GetRawButton(buttonNumber);});
}
