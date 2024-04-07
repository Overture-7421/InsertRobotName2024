// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"

#include <frc/apriltag/AprilTagFieldLayout.h>

class TargetProvider {
public:
	TargetProvider(frc::AprilTagFieldLayout* tagLayout);
	frc::Translation2d GetSpeakerLocation();
	frc::Translation2d GetPassLocation();
private:
	frc::AprilTagFieldLayout* tagLayout;
};
