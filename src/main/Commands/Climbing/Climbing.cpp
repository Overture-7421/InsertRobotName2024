#include "Climbing.h"
#include <exception>

pathplanner::PathConstraints pathfindingConstraints = pathplanner::PathConstraints(
	2.0_mps, 3.0_mps_sq,
	560_deg_per_s, 720_deg_per_s_sq);


SuperStructureState superStructureStartingState{ -15, -60 };
SuperStructureState superStructureTargetState{ 90, -180 };
SuperStructureMoveByDistance::Profile superStructureProfile {superStructureStartingState, superStructureTargetState, 0.55_m};

SupportArmsState supportArmsStartingState{ 0 };
SupportArmsState supportArmsTargetState{ 110 };
SupportArmsMoveByDistance::Profile supportArmsProfile {supportArmsStartingState, supportArmsTargetState, 1_m};

frc2::CommandPtr GoToClimbingLocationPathFind(SuperStructure* superStructure, SupportArms* supportArms, std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {
	return frc2::cmd::Sequence(
			frc2::cmd::Deadline(
				pathplanner::AutoBuilder::pathfindToPoseFlipped(pathToFollow->getStartingDifferentialPose(), pathfindingConstraints),
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

	const auto& poses = pathToFollow->getPathPoses();
	auto targetPos = poses[poses.size() - 1];

	std::function<units::meter_t()> distanceFunction = [=]() {
		if(shouldFlip() && !pathToFollow->preventFlipping){
			return getDistanceToChassis(chassis, pathplanner::GeometryUtil::flipFieldPose(targetPos));
		}else{
			return getDistanceToChassis(chassis, targetPos);
		}
	};

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
		frc2::cmd::RunOnce([=] {
			superStructure->setLowerMotionMagicProfile(500, 1.0, 0.75);
		}),
		SuperStructureCommand(superStructure, { -30, 0 }).ToPtr(),
		WaitForButton(controller, frc::XboxController::Button::kBack),
		frc2::cmd::RunOnce([=] {
			superStructure->resetLowerMotionMagic();
		}),
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
			WaitForButton(controller, frc::XboxController::Button::kBack),
			SetUpJoints(chassis, superStructure, supportArms, climbPathLeft),
			WaitForButton(controller, frc::XboxController::Button::kBack),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ StageLocation::Right, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, supportArms, climbPathRight),
			WaitForButton(controller, frc::XboxController::Button::kBack),
			SetUpJoints(chassis, superStructure, supportArms, climbPathRight),
			WaitForButton(controller, frc::XboxController::Button::kBack),
			ClimbAtLocation(superStructure, controller)
		) },
		std::pair{ StageLocation::Back, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, supportArms, climbPathBack),
			WaitForButton(controller, frc::XboxController::Button::kBack),
			SetUpJoints(chassis, superStructure, supportArms, climbPathBack),
			WaitForButton(controller, frc::XboxController::Button::kBack),
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
		WaitForButton(controller, frc::XboxController::Button::kBack),
		ClimbAtLocation(superStructure, controller)
	);
}