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

static bool shouldFlip() {
	auto alliance = frc::DriverStation::GetAlliance();
	if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
		return true;
	}
	return false;
}

static units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose) {
	return chassis->getOdometry().Translation().Distance(targetPose.Translation());
}

static StageLocation findClosestStageLocation(Chassis* chassis) {
	std::vector<std::pair<StageLocation, units::meter_t>> distancesToStageLocations;
	distancesToStageLocations.reserve(3);

	const std::vector<std::pair<StageLocation, frc::Pose2d>>* stageLocations = &blueStageLocations;

	if (shouldFlip()) {
		stageLocations = &redStageLocations;
	}

	for (auto location : *stageLocations) {
		distancesToStageLocations.push_back(std::pair{ location.first, getDistanceToChassis(chassis, location.second) });
	}

	std::sort(distancesToStageLocations.begin(), distancesToStageLocations.end(), [](auto a, auto b) { return a.second < b.second;});

	return distancesToStageLocations.front().first;
}

