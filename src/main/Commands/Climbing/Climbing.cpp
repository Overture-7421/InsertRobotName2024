#include "Climbing.h"
#include <exception>

#include "main/Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "main/Commands/SupportArmCommand/SupportArmCommand.h"
#include "main/Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "main/Commands/SupportArmsMoveByDistance/SupportArmsMoveByDistance.h"

frc2::CommandPtr GoToClimbingLocationAndSetupJoints(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, StageLocation location) {
	SuperStructureState startingState{ -14, -60 };
	SuperStructureState targetState{ 85, -90 };

	SuperStructureMoveByDistance::Profile profile;
	profile.profileActivationDistance = 1.5_m;
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

	switch (location) {
	case StageLocation::Left:
		pathToFollow = climbPathLeft;
		break;
	case StageLocation::Right:
		pathToFollow = climbPathRight;
		break;
	case StageLocation::Back:
		pathToFollow = climbPathBack;
		break;
	default:
		throw std::logic_error("Tried to go to climbing location that is not implemented");
	}

	auto lastPoint = pathToFollow->getAllPathPoints().at(pathToFollow->getAllPathPoints().size() - 1);
	frc::Pose2d targetPos{ lastPoint.position, pathToFollow->getGoalEndState().getRotation() };

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		2.0_mps, 1.0_mps_sq,
		560_deg_per_s, 720_deg_per_s_sq);

	std::function<units::meter_t()> distanceFunction = [=]() {return getDistanceToChassis(chassis, targetPos);};

	return frc2::cmd::Sequence(
		pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded(pathToFollow->getStartingDifferentialPose()), constraints),
		frc2::cmd::Parallel(
			SuperStructureCommand(superStructure, startingState).ToPtr(),
			SupportArmCommand(supportArms, startingState2).ToPtr()
		),
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::followPath(pathToFollow),
			SuperStructureMoveByDistance(superStructure, profile, distanceFunction).ToPtr(),
			SupportArmsMoveByDistance(supportArms, profile2, distanceFunction).ToPtr()
		)
	);
}

frc2::CommandPtr ClimbAtLocation(SuperStructure* superStructure, frc::XboxController* controller) {
	return frc2::cmd::Sequence(
		SuperStructureCommand(superStructure, {-30, 0}).ToPtr(),
		WaitForButton(controller, frc::XboxController::Button::kA),
		SuperStructureCommand(superStructure, {85, -90}).ToPtr()
	);
}

frc2::CommandPtr Climb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, frc::XboxController* controller) {

	return frc2::cmd::Select<StageLocation>([chassis]() {return findClosestStageLocation(chassis);},
		std::pair{ StageLocation::Left, frc2::cmd::Sequence(
			GoToClimbingLocationAndSetupJoints(chassis, superStructure, supportArms, StageLocation::Left),
			WaitForButton(controller, frc::XboxController::Button::kA),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ StageLocation::Right, frc2::cmd::Sequence(
			GoToClimbingLocationAndSetupJoints(chassis, superStructure, supportArms, StageLocation::Right),
			WaitForButton(controller, frc::XboxController::Button::kA),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ StageLocation::Back, frc2::cmd::Sequence(
			GoToClimbingLocationAndSetupJoints(chassis, superStructure, supportArms, StageLocation::Back),
			WaitForButton(controller, frc::XboxController::Button::kA),
			ClimbAtLocation(superStructure, controller)
		) }
	);
};