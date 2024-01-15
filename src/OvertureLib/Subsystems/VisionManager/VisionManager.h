// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/DriverStation.h>

#include "OvertureLib/subsystems/Swerve/SwerveChassis/SwerveChassis.h"

class VisionManager : public frc2::SubsystemBase {
public:
	VisionManager();
	void setCameraAndLayout(photon::PhotonCamera* camera, frc::AprilTagFieldLayout* tagLayout, frc::Transform3d* cameraToRobot);
	bool checkTagDistance(size_t numberOfTags, double distance);
	void addMeasurementToChassis();
	void updateOdometry();
	std::optional<photon::EstimatedRobotPose> update(frc::Pose2d estimatedPose);
	std::optional<photon::PhotonPipelineResult> getCameraResult();
	bool isPoseEstimatorSet();
	void setPoseEstimator(bool set);
	void Periodic() override;

private:
	/* PhotonVision */
	photon::PhotonCamera* m_Camera;
	frc::AprilTagFieldLayout* m_TagLayout;
	frc::Transform3d* m_CameraToRobot;

	photon::PhotonPoseEstimator* poseEstimator;
	bool poseEstimatorSet = false;

protected:
	/* subsytem */
	SwerveChassis* swerveChassis;
};
