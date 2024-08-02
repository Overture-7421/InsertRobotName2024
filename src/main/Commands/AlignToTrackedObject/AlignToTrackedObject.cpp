#include "AlignToTrackedObject.h"

// TODO: Implement Helpers

frc2::CommandPtr AlignToTrackedObject(Chassis* chassis, photon::PhotonCamera* camera, AlignRobotRelativeHelper* alignHelper) {

	return frc2::cmd::RunOnce([chassis, alignHelper] {
		chassis->enableSpeedHelper(alignHelper);
	}).AndThen(frc2::cmd::Run([=]() mutable {

		const auto result = camera->GetLatestResult();
		if (!result.HasTargets()) {
			alignHelper->setNoteLost();
			return;
		}


		const auto target = result.GetTargets()[0];
		const auto targetYaw = units::degree_t(target.GetYaw());

		if(!alignHelper->isNoteDetected()) {
			alignHelper->setNoteDetected(targetYaw);
		}else{
			alignHelper->setCurrentAngle(targetYaw);
		}
	})).FinallyDo([=] {
		chassis->disableSpeedHelper();
	});
}

frc2::CommandPtr AlignToTrackedObjectFieldOriented(Chassis* chassis, photon::PhotonCamera* camera, AlignFieldRelativeHelper* alignController) {


	return frc2::cmd::RunOnce([chassis, alignController] {
		chassis->enableSpeedHelper(alignController);
	}).AndThen(frc2::cmd::Run([=]() mutable {

		const auto result = camera->GetLatestResult();
		if (!result.HasTargets()) {
			alignController->enable(false);
			return;
		}

		alignController->enable(true);

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
		targetPose = targetPose.RelativeTo(AllignToNoteConstants::CameraOffset.ToPose2d()).RelativeTo(currentRobotPose);

		alignController->setTargetPosition(targetPose.X(), targetPose.Y());


	})).FinallyDo([=] {
		chassis->disableSpeedHelper();
	});
}