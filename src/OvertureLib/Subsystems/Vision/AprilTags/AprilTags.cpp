// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AprilTags.h"
#include <iostream>

AprilTags::AprilTags() {};
void AprilTags::setCameraAndLayout(photon::PhotonCamera* camera, frc::AprilTagFieldLayout* tagLayout, frc::Transform3d* cameraToRobot) {
	this->m_Camera = camera;
	this->m_TagLayout = tagLayout;
	this->m_CameraToRobot = cameraToRobot;
	
	m_TagLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);

	poseEstimatorSet = true;
	poseEstimator = std::make_unique<photon::PhotonPoseEstimator> (
		*m_TagLayout,
		photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
		std::move(photon::PhotonCamera{ APRILTAGS_CAMERA_NAME }),
		*m_CameraToRobot
	);
}

//Check if distance between robot and tag is less than a certain value ;)
bool AprilTags::checkTagDistance(const photon::PhotonPipelineResult& result, size_t numberOfTags, double distance) {

	if (result.GetTargets().size() == numberOfTags) {
		if (result.GetBestTarget().GetBestCameraToTarget().Translation().Distance({0_m, 0_m, 0_m}).value() < distance) {
			return true;
		}
	}

	return false;
}

void AprilTags::addMeasurementToChassis(const photon::PhotonPipelineResult& result) {

	std::optional<photon::EstimatedRobotPose> poseResult = update(result);

	if (poseResult.has_value()) {
		frc::Pose2d poseTo2d = poseResult.value().estimatedPose.ToPose2d();
		swerveChassis->addVisionMeasurement(poseTo2d, poseResult.value().timestamp);
		poseLog.Append(poseTo2d);
	}
}

//Update odometry with vision :0

void AprilTags::updateOdometry() {
	std::optional<photon::PhotonPipelineResult> result = getCameraResult();
	if(!result.has_value()) {
		return;
	}
	photon::PhotonPipelineResult pipelineResult = result.value();

	if (checkTagDistance(pipelineResult, 1, 3.5) || checkTagDistance(pipelineResult, 2, 6.0) || checkTagDistance(pipelineResult, 3, 8.0)) {
		addMeasurementToChassis(pipelineResult);
	}
}

//Get EstimatedRobotPose from PhotonVision
std::optional<photon::EstimatedRobotPose> AprilTags::update(const photon::PhotonPipelineResult& result) {
	return poseEstimator->Update(result);
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


