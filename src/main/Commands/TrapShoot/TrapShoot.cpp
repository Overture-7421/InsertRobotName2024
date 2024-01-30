// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TrapShoot.h"

frc2::CommandPtr GoToShootingLocation(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, Storage* storage, StageLocation location) {
	frc::Pose2d pathToFollow;

	frc::Pose2d shootPathLeft = frc::Pose2d(4.52_m, 4.67_m, 120_deg);
	frc::Pose2d shootPathRight = frc::Pose2d(4.57_m, 3.51_m, -120_deg);
	frc::Pose2d shootPathBack = frc::Pose2d(5.51_m, 4.10_m, 0_deg);

	switch (location) {
	case StageLocation::Left:
		pathToFollow = shootPathLeft;
		break;
	case StageLocation::Right:
		pathToFollow = shootPathRight;
		break;
	case StageLocation::Back:
		pathToFollow = shootPathBack;
		break;
	default:
		throw std::logic_error("Tried to go to shoot location that is not implemented");
	}

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		2.0_mps, 1.0_mps_sq,
		560_deg_per_s, 720_deg_per_s_sq);

	return frc2::cmd::Sequence(
		frc2::cmd::Parallel(
			pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded(pathToFollow), constraints),
			frc2::cmd::RunOnce([=]() {superStructure->setTargetCoord({ 0, 0 });}, { superStructure }),
			frc2::cmd::RunOnce([=]() {shooter->setVelocityVoltage({ 20 });}, { shooter })),
		frc2::cmd::Sequence(
			frc2::cmd::Wait(2_s),
			frc2::cmd::RunOnce([=]() {storage->setVoltage(4.0_V);}, { storage }),
			frc2::cmd::Wait(1_s),
			frc2::cmd::Parallel(
				frc2::cmd::RunOnce([=]() {storage->setVoltage(0_V);}, { storage }),
				frc2::cmd::RunOnce([=]() {shooter->setVelocityVoltage({ 0 });}, { shooter })
			)
		)
	);

}

frc2::CommandPtr TrapShoot(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Shooter* shooter, Storage* storage) {

	return frc2::cmd::Select<StageLocation>([chassis]() {return findClosestStageLocation(chassis);},
		std::pair{ StageLocation::Left, GoToShootingLocation(chassis, superStructure, shooter, storage, StageLocation::Left) },
		std::pair{ StageLocation::Right, GoToShootingLocation(chassis, superStructure, shooter, storage, StageLocation::Right) },
		std::pair{ StageLocation::Back, GoToShootingLocation(chassis, superStructure, shooter, storage, StageLocation::Back) }
	);
}
