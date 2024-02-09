// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "main/Enums/StageLocation.h"
#include "main/Subsystems/Chassis/Chassis.h"
#include <frc/DriverStation.h>

const std::vector<std::pair<StageLocation, frc::Pose2d>> stageLocations{
	{StageLocation::Left, {{4.52_m, 4.67_m}, {120_deg}}},
	{StageLocation::Right,{{4.57_m, 3.51_m}, {-120_deg}}},
	{StageLocation::Back, {{5.51_m, 4.10_m}, {0_deg}}}
};

frc::Pose2d flipPoseIfNeeded(frc::Pose2d pose);

frc::Translation2d flipTranslationIfNeeded(frc::Translation2d translation);

units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose);

StageLocation findClosestStageLocation(Chassis* chassis);