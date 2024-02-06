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

static frc::Pose2d flipPoseIfNeeded(frc::Pose2d pose) {
	static auto alliance = frc::DriverStation::GetAlliance();

	auto shouldFlip = alliance.has_value() && (alliance.value() == frc::DriverStation::Alliance::kRed);
	if (shouldFlip) {
		pose = pathplanner::GeometryUtil::flipFieldPose(pose);
	}
	return pose;
}

static frc::Translation2d flipTranslationIfNeeded(frc::Translation2d translation) {
	static auto alliance = frc::DriverStation::GetAlliance();

	auto shouldFlip = alliance.has_value() && (alliance.value() == frc::DriverStation::Alliance::kRed);
	if (shouldFlip) {
		translation = pathplanner::GeometryUtil::flipFieldPosition(translation);
	}
	return translation;
}

static units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose) {
	return chassis->getOdometry().Translation().Distance(targetPose.Translation());
}

static StageLocation findClosestStageLocation(Chassis* chassis) {
	std::vector<std::pair<StageLocation, units::meter_t>> distancesToStageLocations;
	distancesToStageLocations.reserve(3);


	for (auto location : stageLocations) {
		distancesToStageLocations.push_back(std::pair{ location.first, getDistanceToChassis(chassis, flipPoseIfNeeded(location.second)) });
	}

	std::sort(distancesToStageLocations.begin(), distancesToStageLocations.end(), [](auto a, auto b) { return a.second < b.second;});

	return distancesToStageLocations.front().first;
}
