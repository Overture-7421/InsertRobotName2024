#include "UtilityFunctions.h"

bool shouldFlip() {
	auto alliance = frc::DriverStation::GetAlliance();
	if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
		return true;
	}
	return false;
}

units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose) {
	return chassis->getOdometry().Translation().Distance(targetPose.Translation());
}

StageLocation findClosestStageLocation(Chassis* chassis) {
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
