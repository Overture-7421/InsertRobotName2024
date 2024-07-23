// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionAmpCommand.h"
#include <frc2/command/ParallelCommandGroup.h>
#include "Commands/AmpCommand/AmpCommand.h"

// TODO: Implement Helpers
frc2::CommandPtr VisionAmpCommand(SuperStructure* superStucture, Shooter* shooter) {

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		3.0_mps, 3.0_mps_sq,
		560_deg_per_s, 720_deg_per_s_sq);

	return frc2::cmd::Sequence(
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::pathfindToPoseFlipped({ 1.80_m, 7.59_m, {-90_deg} }, constraints),
			superStucture->superStructureCommand(SuperStructureConstants::AmpState),
			shooter->shooterCommand(ShooterConstants::AmpScoreSpeed)
		),
		AmpCommand(superStucture, shooter)
	);
};

// frc2::CommandPtr VisionAmpCommand(Chassis* chassis) {
// 	frc::ProfiledPIDController<units::meter> alignXController{ 0.1, 0.0, 0.0, {2_mps, 5_mps_sq}, RobotConstants::LoopTime };
// 	frc::SlewRateLimiter<units::meters_per_second> vxLimiter{ 5.0_mps_sq };

// 	double maxOutput = 0.5;

// 	return frc2::cmd::Run([&] {
// 		double output = alignXController.Calculate(chassis->getEstimatedPose().Translation().X(), pathplanner::GeometryUtil::flipFieldPosition({ 1.80_m, 7.59_m }).X());
// 		output = std::clamp(output, -maxOutput, maxOutput);

// 		// chassis->setPositionTarget(vxLimiter.Calculate(units::meters_per_second_t(0)), units::meters_per_second_t(output));
// 	}).BeforeStarting([=] {
// 		// chassis->setPositionAssist(true);
// 		// chassis->setHeadingOverride(true);
// 		// chassis->setTargetHeading({ -90_deg });

// 	}).AndThen([&] {
// 		// chassis->setPositionAssist(false);
// 		// chassis->setHeadingOverride(false);
// 	});
// };