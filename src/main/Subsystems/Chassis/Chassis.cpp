// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Chassis.h"

Chassis::Chassis() : SwerveChassis() {
#ifndef __FRC_ROBORIO__
	frontLeft.setFFConstants(0.040347_V, 1.93_V, 0.63363_V);
	frontRight.setFFConstants(0.26831_V, 1.9683_V, 0.1204_V);
	backLeft.setFFConstants(0.0089267_V, 1.8401_V, 0.77189_V);
	backRight.setFFConstants(0.15117_V, 1.9912_V, 0.032941_V);
#else
	frontLeft.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
	frontRight.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
	backLeft.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
	backRight.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
#endif
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
