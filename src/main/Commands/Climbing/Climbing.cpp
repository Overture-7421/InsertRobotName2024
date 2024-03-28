#include "Climbing.h"
#include "main/Commands/StorageCommand/StorageCommand.h"
#include "main/Commands/ShooterCommand/ShooterCommand.h"
#include "main/Commands/ClosedCommand/ClosedCommand.h"
#include "main/Commands/FreeSupportArms/FreeSupportArms.h"
#include <exception>

pathplanner::PathConstraints pathfindingConstraints = pathplanner::PathConstraints(
	2.0_mps, 2.5_mps_sq,
	560_deg_per_s, 720_deg_per_s_sq);

int checkpointButtonId = frc::XboxController::Button::kBack;

SuperStructureState superStructureStartingState{ -15, -60 };
SuperStructureState superStructureTargetState{ 85, -90 };
SuperStructureMoveByDistance::Profile superStructureProfile{ superStructureStartingState, superStructureTargetState, 1.25_m };

units::second_t storageTrapScoreWait = 1_s;

frc2::CommandPtr GoToClimbingLocationPathFind(SuperStructure* superStructure, std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {
	return frc2::cmd::Sequence(
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::pathfindToPoseFlipped(pathToFollow->getStartingDifferentialPose(), pathfindingConstraints),
			SuperStructureCommand(superStructure, superStructureStartingState).ToPtr()
		)
	);
}

frc2::CommandPtr GoToClimbingLocationOnTheFly(std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {
	return pathplanner::AutoBuilder::followPath(pathToFollow);
}

frc2::CommandPtr SetUpJoints(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {

	const auto& poses = pathToFollow->getPathPoses();
	auto targetPos = poses[poses.size() - 1];

	std::function<units::meter_t()> distanceFunction = [=]() {
		if (isRedAlliance() && !pathToFollow->preventFlipping) {
			return getDistanceToChassis(chassis, pathplanner::GeometryUtil::flipFieldPose(targetPos));
		} else {
			return getDistanceToChassis(chassis, targetPos);
		}
	};

	return frc2::cmd::Sequence(
		SuperStructureCommand(superStructure, superStructureStartingState).ToPtr(),
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::followPath(pathToFollow),
			SuperStructureMoveByDistance(superStructure, superStructureProfile, distanceFunction).ToPtr(),
			FreeSupportArms(supportArms, 135.00).ToPtr().Repeatedly()
		)
	);
}


frc2::CommandPtr ClimbAtLocation(SuperStructure* superStructure, Shooter* shooter, Storage* storage, frc::XboxController* controller) {
	return frc2::cmd::Sequence(
		SuperStructureCommand(superStructure, SuperStructureConstants::GroundGrabState).ToPtr().WithTimeout(1_s),
		WaitForButton(controller, checkpointButtonId),
		SuperStructureCommand(superStructure, { 87, -82 }).ToPtr().WithTimeout(1_s),
		frc2::cmd::RunOnce([=] { shooter->setVoltage(7.0);}),
		WaitForButton(controller, checkpointButtonId),
		StorageCommand(storage, 3_V).ToPtr(),
		frc2::cmd::WaitUntil([=] {return !storage->isNoteOnForwardSensor();}),
		WaitForButton(controller, checkpointButtonId),
		SuperStructureCommand(superStructure, SuperStructureConstants::GroundGrabState).ToPtr(),
		frc2::cmd::RunOnce([=] { shooter->setVoltage(0.0);})
	);
}

frc2::CommandPtr AutoClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Storage* storage, Shooter* shooter, frc::XboxController* controller) {
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathLeft = pathplanner::PathPlannerPath::fromPathFile("Climb Left");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathRight = pathplanner::PathPlannerPath::fromPathFile("Climb Right");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathBack = pathplanner::PathPlannerPath::fromPathFile("Climb Back");

	return frc2::cmd::Select<StageLocation>([chassis]() {return findClosestStageLocation(chassis);},
		std::pair{ StageLocation::Left, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, climbPathLeft),
			WaitForButton(controller, checkpointButtonId),
			SetUpJoints(chassis, superStructure, supportArms, climbPathLeft),
			WaitForButton(controller, checkpointButtonId),
			ClimbAtLocation(superStructure, shooter, storage, controller)
		) },
		std::pair{ StageLocation::Right, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, climbPathRight),
			WaitForButton(controller, checkpointButtonId),
			SetUpJoints(chassis, superStructure, supportArms, climbPathRight),
			WaitForButton(controller, checkpointButtonId),
			ClimbAtLocation(superStructure, shooter, storage, controller)
		) },
		std::pair{ StageLocation::Back, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, climbPathBack),
			WaitForButton(controller, checkpointButtonId),
			SetUpJoints(chassis, superStructure, supportArms, climbPathBack),
			WaitForButton(controller, checkpointButtonId),
			ClimbAtLocation(superStructure, shooter, storage,controller)
		) }
	);
};

frc2::CommandPtr ManualClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, AprilTagCamera* aprilTagCamera, Storage* storage, Shooter* shooter, frc::XboxController* controller) {
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathManual = pathplanner::PathPlannerPath::fromPathFile("Climb Manual");
	climbPathManual->preventFlipping = true;

	return frc2::cmd::Sequence(
		frc2::cmd::RunOnce([=]() {
		aprilTagCamera->setPoseEstimator(false);
		chassis->resetOdometry(climbPathManual->getStartingDifferentialPose());
	}),
		SetUpJoints(chassis, superStructure, supportArms, climbPathManual),
		WaitForButton(controller, checkpointButtonId),
		ClimbAtLocation(superStructure, shooter, storage, controller)
	);
}