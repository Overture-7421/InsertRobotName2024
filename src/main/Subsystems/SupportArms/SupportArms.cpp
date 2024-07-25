// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArms.h"

SupportArms::SupportArms() {};

void SupportArms::SetAngle(double angle) {
	leftServo.SetAngle(angle);
	rightServo.SetAngle(angle);
};

void SupportArms::Periodic() {};

frc2::CommandPtr SupportArms::freeArmsCommand(double angle) {
	return std::move(frc2::cmd::RunOnce([this, angle] {this->SetAngle(angle);}));
}