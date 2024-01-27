#include "Climbing.h"
#include <exception>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "main/Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "main/Commands/SupportArmsMoveByDistance/SupportArmsMoveByDistance.h"

#include <iostream>

frc::Pose2d flipPoseIfNeeded(frc::Pose2d pose) {
	auto alliance = frc::DriverStation::GetAlliance();

	auto shouldFlip = alliance.has_value() && (alliance.value() == frc::DriverStation::Alliance::kRed);
	if (shouldFlip) {
		pose = pathplanner::GeometryUtil::flipFieldPose(pose);
	}
	return pose;
}

units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose) {
	return chassis->getOdometry().Translation().Distance(flipPoseIfNeeded(targetPose).Translation());
}

ClimbingLocation findClosestClimbingLocation(Chassis* chassis) {
	std::vector<std::pair<ClimbingLocation, units::meter_t>> distancesToClimbingLocations;
	distancesToClimbingLocations.reserve(3);


	for (auto location : climbingLocations) {
		distancesToClimbingLocations.push_back(std::pair{ location.first, getDistanceToChassis(chassis, location.second) });
	}

	std::sort(distancesToClimbingLocations.begin(), distancesToClimbingLocations.end(), [](auto a, auto b) { return a.second < b.second;});
	return distancesToClimbingLocations.front().first;
}

frc2::CommandPtr Climb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, frc::XboxController* controller) {

	SuperStructureState startingState{ -4, -100 };
	SuperStructureState targetState{ 90, -90 };

	SuperStructureMoveByDistance::Profile profile;
	profile.profileActivationDistance = 1_m;
	profile.startingState = startingState;
	profile.targetState = targetState;

	SupportArmsState startingState2{ 0 };
	SupportArmsState targetState2{ 120 };

	SupportArmsMoveByDistance::Profile profile2;
	profile2.profileActivationDistance = 1_m;
	profile2.startingState = startingState2;
	profile2.targetState = targetState2;


	std::shared_ptr<pathplanner::PathPlannerPath> climbPathLeft = pathplanner::PathPlannerPath::fromPathFile("AMP Climb Left");
	std::shared_ptr<pathplanner::PathPlannerPath> climbPathRight = pathplanner::PathPlannerPath::fromPathFile("AMP Climb Right");
	std::shared_ptr<pathplanner::PathPlannerPath> climbPathBack = pathplanner::PathPlannerPath::fromPathFile("AMP Climb Back");

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		2.0_mps, 1.0_mps_sq,
		560_deg_per_s, 720_deg_per_s_sq);

	return frc2::cmd::Select<ClimbingLocation>([chassis]() {return findClosestClimbingLocation(chassis);},
		std::pair{ ClimbingLocation::Left, frc2::cmd::Sequence(
			pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded(climbPathLeft->getStartingDifferentialPose()), constraints),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord(startingState);}, {superStructure}),
			frc2::cmd::Wait(3_s),
			frc2::cmd::Parallel(
				pathplanner::AutoBuilder::followPath(climbPathLeft),
				SuperStructureMoveByDistance(superStructure, profile, [=]() {return getDistanceToChassis(chassis, climbingLocations[0].second);}).ToPtr(),
				SupportArmsMoveByDistance(supportArms, profile2, [=]() {return getDistanceToChassis(chassis, climbingLocations[0].second);}).ToPtr()
			),
			frc2::cmd::WaitUntil([=]() {return controller->GetAButton();}),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({-30 , 90});}, {superStructure}),
			frc2::cmd::Wait(3_s),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({90, 90});}, {superStructure}),
			frc2::cmd::Wait(3_s)
		) },
		std::pair{ ClimbingLocation::Right, frc2::cmd::Sequence(
			pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded(climbPathRight->getStartingDifferentialPose()), constraints),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord(startingState);}, {superStructure}),
			frc2::cmd::Wait(3_s),
			frc2::cmd::Parallel(
				pathplanner::AutoBuilder::followPath(climbPathRight),
				SuperStructureMoveByDistance(superStructure, profile, [=]() {return getDistanceToChassis(chassis, climbingLocations[1].second);}).ToPtr(),
				SupportArmsMoveByDistance(supportArms, profile2, [=]() {return getDistanceToChassis(chassis, climbingLocations[1].second);}).ToPtr()
			),
			frc2::cmd::WaitUntil([=]() {return controller->GetAButton();}),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({-30 , 90});}, {superStructure}),
			frc2::cmd::Wait(3_s),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({90, 90});}, {superStructure}),
			frc2::cmd::Wait(3_s)
		) },
		std::pair{ ClimbingLocation::Back, frc2::cmd::Sequence(
			pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded(climbPathBack->getStartingDifferentialPose()), constraints),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord(startingState);}, {superStructure}),
			frc2::cmd::Wait(3_s),
			frc2::cmd::Parallel(
				pathplanner::AutoBuilder::followPath(climbPathBack),
				SuperStructureMoveByDistance(superStructure, profile, [=]() {return getDistanceToChassis(chassis, climbingLocations[2].second);}).ToPtr(),
				SupportArmsMoveByDistance(supportArms, profile2, [=]() {return getDistanceToChassis(chassis, climbingLocations[2].second);}).ToPtr()
			),
			frc2::cmd::WaitUntil([=]() {return controller->GetAButton();}),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({-30 , 90});}, {superStructure}),
			frc2::cmd::Wait(3_s),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({90, 90});}, {superStructure}),
			frc2::cmd::Wait(3_s)
		) }
	);
};

std::ostream& operator<< (std::ostream& os, ClimbingLocation location) {
	switch (location) {
	case ClimbingLocation::Left: return os << "Left";
	case ClimbingLocation::Right: return os << "Right";
	case ClimbingLocation::Back: return os << "Back";
		// omit default case to trigger compiler warning for missing cases
	};
	return os << static_cast<std::uint16_t>(location);
}