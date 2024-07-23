#include "AlignToTrackedObject.h"

// TODO: Implement Helpers

frc2::CommandPtr AlignToTrackedObject(Chassis* chassis, photon::PhotonCamera* camera) {
	AlignRobotRelativeHelper alignHelper;

	return frc2::cmd::RunOnce([chassis] {
		chassis->disableSpeedHelper();
	}).AndThen(frc2::cmd::Run([=]() mutable {

		const auto result = camera->GetLatestResult();
		if (!result.HasTargets()) {
			chassis->disableSpeedHelper();
			return;
		}
		chassis->enableSpeedHelper(&alignHelper);

		const auto target = result.GetTargets()[0];
		alignHelper.setCurrentAngle(units::degree_t(target.GetYaw()));

	})).FinallyDo([=] {
		chassis->disableSpeedHelper();
	});
}

frc2::CommandPtr AlignToTrackedObjectFieldOriented(Chassis* chassis, photon::PhotonCamera* camera) {

	AlignFieldRelativeHelper alignController{ chassis };

	return frc2::cmd::RunOnce([chassis] {
		chassis->disableSpeedHelper();
	}).AndThen(frc2::cmd::Run([=]() mutable {

		const auto result = camera->GetLatestResult();
		if (!result.HasTargets()) {
			chassis->disableSpeedHelper();
			return;
		}

		chassis->enableSpeedHelper(&alignController);

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

		alignController.setTargetPosition(targetPose.X(), targetPose.Y());


	})).FinallyDo([=] {
		chassis->disableSpeedHelper();
	});
}