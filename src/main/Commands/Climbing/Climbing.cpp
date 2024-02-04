#include "Climbing.h"
#include <exception>

pathplanner::PathConstraints pathfindingConstraints = pathplanner::PathConstraints(
	1.5_mps, 3.0_mps_sq,
	560_deg_per_s, 720_deg_per_s_sq);


SuperStructureState superStructureStartingState{ -14, -60 };
SuperStructureState superStructureTargetState{ 85, -90 };
SuperStructureMoveByDistance::Profile superStructureProfile {superStructureStartingState, superStructureTargetState, 1.5_m};

SupportArmsState supportArmsStartingState{ 0 };
SupportArmsState supportArmsTargetState{ 110 };
SupportArmsMoveByDistance::Profile supportArmsProfile {supportArmsStartingState, supportArmsTargetState, 1_m};


frc2::CommandPtr GoToClimbingLocationPathFind(SuperStructure* superStructure, SupportArms* supportArms, std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {
	return frc2::cmd::Sequence(
			frc2::cmd::Deadline(
				pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded(pathToFollow->getStartingDifferentialPose()), pathfindingConstraints),
				SuperStructureCommand(superStructure, superStructureStartingState).ToPtr(),
				SupportArmCommand(supportArms, supportArmsStartingState).ToPtr()
			),
			frc2::cmd::Wait(0.5_s)
		);
}

frc2::CommandPtr GoToClimbingLocationOnTheFly(std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {
	return pathplanner::AutoBuilder::followPath(pathToFollow);
}

frc2::CommandPtr SetUpJoints(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {

	auto lastPoint = pathToFollow->getAllPathPoints().at(pathToFollow->getAllPathPoints().size() - 1);
	frc::Pose2d targetPos{ lastPoint.position, pathToFollow->getGoalEndState().getRotation() };
	
	if(!pathToFollow->preventFlipping){
		targetPos = flipPoseIfNeeded(targetPos);
	}

	std::function<units::meter_t()> distanceFunction = [=]() {return getDistanceToChassis(chassis, targetPos);};

	return frc2::cmd::Sequence(
		frc2::cmd::Parallel(
			SuperStructureCommand(superStructure, superStructureStartingState).ToPtr(),
			SupportArmCommand(supportArms, supportArmsStartingState).ToPtr()
		),
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::followPath(pathToFollow),
			SuperStructureMoveByDistance(superStructure, superStructureProfile, distanceFunction).ToPtr(),
			SupportArmsMoveByDistance(supportArms, supportArmsProfile, distanceFunction).ToPtr()
		)
	);
}


frc2::CommandPtr ClimbAtLocation(SuperStructure* superStructure, frc::XboxController* controller) {
	return frc2::cmd::Sequence(
		SuperStructureCommand(superStructure, { -30, 0 }).ToPtr(),
		WaitForButton(controller, frc::XboxController::Button::kA),
		SuperStructureCommand(superStructure, { 85, -90 }).ToPtr()
	);
}

frc2::CommandPtr AutoClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, frc::XboxController* controller) {
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathLeft = pathplanner::PathPlannerPath::fromPathFile("Climb Left");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathRight = pathplanner::PathPlannerPath::fromPathFile("Climb Right");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathBack = pathplanner::PathPlannerPath::fromPathFile("Climb Back");

	return frc2::cmd::Select<StageLocation>([chassis]() {return findClosestStageLocation(chassis);},
		std::pair{ StageLocation::Left, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, supportArms, climbPathLeft),
			SetUpJoints(chassis, superStructure, supportArms, climbPathLeft),
			WaitForButton(controller, frc::XboxController::Button::kA),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ StageLocation::Right, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, supportArms, climbPathRight),
			SetUpJoints(chassis, superStructure, supportArms, climbPathRight),
			WaitForButton(controller, frc::XboxController::Button::kA),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ StageLocation::Back, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, supportArms, climbPathBack),
			SetUpJoints(chassis, superStructure, supportArms, climbPathBack),
			WaitForButton(controller, frc::XboxController::Button::kA),
			ClimbAtLocation(superStructure, controller)
		) }
	);
};

frc2::CommandPtr ManualClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, AprilTagCamera* aprilTagCamera, frc::XboxController* controller) {
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathManual = pathplanner::PathPlannerPath::fromPathFile("Climb Manual");
	climbPathManual->preventFlipping = true;

	return frc2::cmd::Sequence(
		frc2::cmd::RunOnce([=]() {
			aprilTagCamera->setPoseEstimator(false);
			chassis->resetOdometry(climbPathManual->getStartingDifferentialPose());
		}),
		SetUpJoints(chassis, superStructure, supportArms, climbPathManual),
		WaitForButton(controller, frc::XboxController::Button::kA),
		ClimbAtLocation(superStructure, controller)
	);
}