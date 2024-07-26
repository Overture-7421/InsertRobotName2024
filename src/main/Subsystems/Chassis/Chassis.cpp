// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Chassis.h"

Chassis::Chassis() : SwerveChassis() {
	configureSwerveBase();
}

void Chassis::setAllianceColor() {
	if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
		m_AllianceColor = -1;
	} else {
		m_AllianceColor = 1;
	}
}

void Chassis::driveFieldRelative(frc::ChassisSpeeds speeds) {
	setTargetSpeeds(
		frc::ChassisSpeeds::Discretize(
			frc::ChassisSpeeds::FromFieldRelativeSpeeds(
				{
					speeds.vx * m_AllianceColor,
					speeds.vy * m_AllianceColor,
					speeds.omega
				},
				getEstimatedPose().Rotation()
			), RobotConstants::LoopTime
		)
	);
}
