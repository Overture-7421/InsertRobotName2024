// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/subsystems/VisionManager/VisionManager.h"
#include "main/subsystems/Chassis/Chassis.h"

class Vision : public VisionManager {
public:
	Vision(Chassis* chassis);
private:
	photon::PhotonCamera camera{ "Arducam_OV9281_USB_Camera" };
	frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo) };
	frc::Transform3d cameraToRobot{ {0_in, 0_in, 0_in}, {0_deg, 0_deg, 0_deg} };
};
