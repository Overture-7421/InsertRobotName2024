// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h"
#include "main/Subsystems/Chassis/Chassis.h"



class AprilTagCamera : public AprilTags {
public:
	AprilTagCamera(Chassis* chassis);
private:

	photon::PhotonCamera camera{ APRILTAGS_CAMERA_NAME };
	frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo) };
	frc::Transform3d cameraToRobot{ {0_in, 0_in, 0_in}, {0_deg, 0_deg, 0_deg} };
};
