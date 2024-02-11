#include "UtilityFunctions.h"

frc::Pose2d flipPoseIfNeeded(frc::Pose2d pose) {
	static auto alliance = frc::DriverStation::GetAlliance();

	auto shouldFlip = alliance.has_value() && (alliance.value() == frc::DriverStation::Alliance::kRed);
	if (shouldFlip) {
		pose = pathplanner::GeometryUtil::flipFieldPose(pose);
	}
	return pose;
}

frc::Translation2d flipTranslationIfNeeded(frc::Translation2d translation) {
	static auto alliance = frc::DriverStation::GetAlliance();

	auto shouldFlip = alliance.has_value() && (alliance.value() == frc::DriverStation::Alliance::kRed);
	if (shouldFlip) {
		translation = pathplanner::GeometryUtil::flipFieldPosition(translation);
	}
	return translation;
}

units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose) {
	return chassis->getOdometry().Translation().Distance(targetPose.Translation());
}

StageLocation findClosestStageLocation(Chassis* chassis) {
	std::vector<std::pair<StageLocation, units::meter_t>> distancesToStageLocations;
	distancesToStageLocations.reserve(3);


	for (auto location : stageLocations) {
		distancesToStageLocations.push_back(std::pair{ location.first, getDistanceToChassis(chassis, flipPoseIfNeeded(location.second)) });
	}

	std::sort(distancesToStageLocations.begin(), distancesToStageLocations.end(), [](auto a, auto b) { return a.second < b.second;});

	return distancesToStageLocations.front().first;
}
