#include "Climbing.h"
#include <exception>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "main/Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "main/Commands/SupportArmsMoveByDistance/SupportArmsMoveByDistance.h"

// TODO: Update superstructure and support arm commands to not wait 3s for them to end, use real commands when they are completed.

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

frc2::CommandPtr WaitForCheckpointButton(frc::XboxController* controller){
	return frc2::cmd::WaitUntil([=]() {return controller->GetAButton();});
}

frc2::CommandPtr GoToClimbingLocationAndSetupJoints(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, ClimbingLocation location){
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


	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathLeft = pathplanner::PathPlannerPath::fromPathFile("AMP Climb Left");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathRight = pathplanner::PathPlannerPath::fromPathFile("AMP Climb Right");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathBack = pathplanner::PathPlannerPath::fromPathFile("AMP Climb Back");

	std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow;
	frc::Pose2d targetPos;

	switch (location) {
		case ClimbingLocation::Left:
			pathToFollow = climbPathLeft;
			targetPos = climbingLocations[0].second;
			break;
		case ClimbingLocation::Right:
			pathToFollow = climbPathRight;
			targetPos = climbingLocations[1].second;
			break;
		case ClimbingLocation::Back:
			pathToFollow = climbPathBack;
			targetPos = climbingLocations[2].second;
			break;
		default:
			throw std::logic_error("Tried to go to climbing location that is not implemented");
	}

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
	2.0_mps, 1.0_mps_sq,
	560_deg_per_s, 720_deg_per_s_sq);

	std::function<units::meter_t()> distanceFunction = [=]() {return getDistanceToChassis(chassis, targetPos);};

	return frc2::cmd::Sequence(
		pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded(pathToFollow->getStartingDifferentialPose()), constraints),
		frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord(startingState);}, {superStructure}),
		frc2::cmd::Wait(3_s),
		frc2::cmd::Parallel(
			pathplanner::AutoBuilder::followPath(pathToFollow),
			SuperStructureMoveByDistance(superStructure, profile, distanceFunction).ToPtr(),
			SupportArmsMoveByDistance(supportArms, profile2, distanceFunction).ToPtr()
		)
	);
}

frc2::CommandPtr ClimbAtLocation(SuperStructure* superStructure, frc::XboxController* controller) {
	return frc2::cmd::Sequence(
		frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({-30 , 0});}, {superStructure}),
		frc2::cmd::Wait(3_s),
		WaitForCheckpointButton(controller),
		frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({85, -90});}, {superStructure}),
		frc2::cmd::Wait(3_s)
	);
}

frc2::CommandPtr Climb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, frc::XboxController* controller) {

	return frc2::cmd::Select<ClimbingLocation>([chassis]() {return findClosestClimbingLocation(chassis);},
		std::pair{ ClimbingLocation::Left, frc2::cmd::Sequence(
			GoToClimbingLocationAndSetupJoints(chassis, superStructure, supportArms, ClimbingLocation::Left),
			WaitForCheckpointButton(controller),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ ClimbingLocation::Right, frc2::cmd::Sequence(
			GoToClimbingLocationAndSetupJoints(chassis, superStructure, supportArms, ClimbingLocation::Right),
			WaitForCheckpointButton(controller),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ ClimbingLocation::Back, frc2::cmd::Sequence(
			GoToClimbingLocationAndSetupJoints(chassis, superStructure, supportArms, ClimbingLocation::Back),
			WaitForCheckpointButton(controller),
			ClimbAtLocation(superStructure, controller)
		) }
	);
};