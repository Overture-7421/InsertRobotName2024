// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "main/Enums/StageLocation.h"
#include "main/Subsystems/Chassis/Chassis.h"
#include <frc/DriverStation.h>

const std::vector<std::pair<StageLocation, frc::Pose2d>> blueStageLocations{
	{StageLocation::Left, {{4.52_m, 4.67_m}, {120_deg}}},
	{StageLocation::Right,{{4.57_m, 3.51_m}, {-120_deg}}},
	{StageLocation::Back, {{5.51_m, 4.10_m}, {0_deg}}}
};

const std::vector<std::pair<StageLocation, frc::Pose2d>> redStageLocations{
	{StageLocation::Left,  pathplanner::GeometryUtil::flipFieldPose(blueStageLocations[0].second)},
	{StageLocation::Right, pathplanner::GeometryUtil::flipFieldPose(blueStageLocations[1].second)},
	{StageLocation::Back,  pathplanner::GeometryUtil::flipFieldPose(blueStageLocations[2].second)}
};

bool isRedAlliance();
units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose);
StageLocation findClosestStageLocation(Chassis* chassis);
