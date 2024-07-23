#include "AlignToTrackedObject.h"

// TODO: Implement Helpers

frc2::CommandPtr AlignToTrackedObject(Chassis* chassis, photon::PhotonCamera* camera) {

	frc::ProfiledPIDController<units::degree> alignController{ 0.1, 0.0, 0.0, {120_deg_per_s, 360_deg_per_s_sq}, RobotConstants::LoopTime };
	frc::SlewRateLimiter<units::meters_per_second> vyLimiter{ 15.0_mps_sq };
	

	return frc2::cmd::RunOnce([chassis] {
		// chassis->setVyOverride(false);
	}).AndThen(frc2::cmd::Run([=]() mutable {

		const auto result = camera->GetLatestResult();
		if (!result.HasTargets()) {
			// chassis->setVyOverride(false);
			// chassis->setVyTarget(vyLimiter.Calculate(units::meters_per_second_t(0)));
			return;
		}
		// chassis->setVyOverride(true);

		const auto target = result.GetTargets()[0];
		double targetVy = alignController.Calculate(units::degree_t(target.GetYaw()), units::degree_t(0));

		// frc::SmartDashboard::PutNumber("AlignToVisionTarget/TargetVy", targetVy);
		// chassis->setVyTarget(vyLimiter.Calculate(units::meters_per_second_t(targetVy)));
	})).FinallyDo([=] {
		// chassis->setVyOverride(false);
		// chassis->setVyTarget(units::meters_per_second_t(0));
	});
}

frc2::CommandPtr AlignToTrackedObjectFieldOriented(Chassis* chassis, photon::PhotonCamera* camera) {

	frc::ProfiledPIDController<units::meter> alignXController{ 0.1, 0.0, 0.0, {2_mps, 5_mps_sq}, RobotConstants::LoopTime };
	frc::ProfiledPIDController<units::meter> alignYController{ 0.1, 0.0, 0.0, {2_mps, 5_mps_sq}, RobotConstants::LoopTime };

	frc::SlewRateLimiter<units::meters_per_second> vyLimiter{ 5.0_mps_sq };
	frc::SlewRateLimiter<units::meters_per_second> vxLimiter{ 5.0_mps_sq };

	return frc2::cmd::RunOnce([chassis] {
		// chassis->setPositionAssist(false);
	}).AndThen(frc2::cmd::Run([=]() mutable {

		const auto result = camera->GetLatestResult();
		if (!result.HasTargets()) {
			// chassis->setVyOverride(false);
			// chassis->setPositionTarget(vyLimiter.Calculate(units::meters_per_second_t(0)), vxLimiter.Calculate(units::meters_per_second_t(0)));
			return;
		}
		// chassis->setVyOverride(true);

		const auto target = result.GetTargets()[0];
		auto corners = target.GetDetectedCorners();

		std::sort(corners.begin(), corners.end(), [](auto a, auto b) {
			return a.first < b.first;
		});

		auto leftCorner = corners[0];
		auto rightCorner = corners[3];
		double targetWidth = std::abs(leftCorner.first - rightCorner.first);

		double angle = AllignToNoteConstants::PixelsToAngle[targetWidth];
		units::meter_t distance = AllignToNoteConstants::NoteWidth / std::tan(angle);

		double yaw = target.GetYaw();
		frc::Pose2d targetPose{ distance * std::cos(yaw), distance * std::sin(yaw), 0_deg };
		frc::Pose2d currentRobotPose = chassis->getEstimatedPose();
		targetPose = targetPose.RelativeTo(AllignToNoteConstants::CameraOffset).RelativeTo(currentRobotPose);

		double targetVy = alignYController.Calculate(currentRobotPose.Translation().Y(), targetPose.Translation().Y());
		double targetVx = alignXController.Calculate(currentRobotPose.Translation().X(), targetPose.Translation().X());

		targetVy = std::clamp(targetVy, AllignToNoteConstants::maxOutput, -AllignToNoteConstants::maxOutput);
		targetVx = std::clamp(targetVx, AllignToNoteConstants::maxOutput, -AllignToNoteConstants::maxOutput);

		// chassis->setPositionTarget(vyLimiter.Calculate(units::meters_per_second_t(targetVy)), vxLimiter.Calculate(units::meters_per_second_t(targetVx)));
	})).FinallyDo([=] {
		// chassis->setPositionAssist(false);
		// chassis->setPositionTarget(units::meters_per_second_t(0), units::meters_per_second_t(0));
	});
}