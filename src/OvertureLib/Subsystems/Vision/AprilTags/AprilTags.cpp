// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AprilTags.h"

AprilTags::AprilTags() {};
void AprilTags::setCameraAndLayout(photon::PhotonCamera* camera, frc::AprilTagFieldLayout* tagLayout, frc::Transform3d* cameraToRobot) {
	this->m_Camera = camera;
	this->m_TagLayout = tagLayout;
	this->m_CameraToRobot = cameraToRobot;

	m_TagLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);

	poseEstimatorSet = true;
	poseEstimator = new photon::PhotonPoseEstimator{
		*m_TagLayout,
		photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
		std::move(photon::PhotonCamera{ APRILTAGS_CAMERA_NAME }),
		*m_CameraToRobot
	};
}

//Check if distance between robot and tag is less than a certain value ;)
bool AprilTags::checkTagDistance(size_t numberOfTags, double distance) {
	std::optional<photon::PhotonPipelineResult> result = getCameraResult();

	if (result.has_value()) {
		photon::PhotonPipelineResult resultValue = result.value();
		if (resultValue.GetTargets().size() == numberOfTags) {
			if (resultValue.GetBestTarget().GetBestCameraToTarget().X().value() < distance) {
				return true;
			}
		}
	}

	return false;
}

void AprilTags::addMeasurementToChassis() {
	std::optional<photon::EstimatedRobotPose> poseResult = update(swerveChassis->getOdometry());

	if (poseResult.has_value()) {
		frc::Pose2d poseTo2d = poseResult.value().estimatedPose.ToPose2d();
		swerveChassis->addVisionMeasurement({ poseTo2d.X(), poseTo2d.Y(), swerveChassis->getOdometry().Rotation() }, poseResult.value().timestamp);
	}
}

//Update odometry with vision :0

void AprilTags::updateOdometry() {
	if (checkTagDistance(1, 16.00) || checkTagDistance(2, 16.00) || checkTagDistance(3, 6.00)) {
		addMeasurementToChassis();
	}
}

//Get EstimatedRobotPose from PhotonVision
std::optional<photon::EstimatedRobotPose> AprilTags::update(frc::Pose2d estimatedPose) {
	return poseEstimator->Update();
}

//Get PhotonPipeResult from PhotonVision
std::optional<photon::PhotonPipelineResult> AprilTags::getCameraResult() {
	return m_Camera->GetLatestResult();
}

//Check if poseEstimator is set
bool AprilTags::isPoseEstimatorSet() {
	return poseEstimatorSet;
}

void AprilTags::setPoseEstimator(bool set) {
	poseEstimatorSet = set;
}

void AprilTags::Periodic() {
	// frc::SmartDashboard::PutBoolean("Set Camara", isPoseEstimatorSet());
	if (isPoseEstimatorSet()) {
		updateOdometry();
	}
}


