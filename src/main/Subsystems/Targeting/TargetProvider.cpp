// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TargetProvider.h"

#include <pathplanner/lib/util/GeometryUtil.h>

const frc::Translation2d PassLocation{ 1.03_m, 7.31_m };
const frc::Translation2d SpeakerTargetOffset{ 0_m, 0_m };

TargetProvider::TargetProvider(frc::AprilTagFieldLayout* tagLayout) {
	this->tagLayout = tagLayout;
}

frc::Translation2d TargetProvider::GetSpeakerLocation() {
	if (isRedAlliance()) {
		return tagLayout->GetTagPose(4).value().ToPose2d().Translation() + SpeakerTargetOffset;
	} else {
		return tagLayout->GetTagPose(7).value().ToPose2d().Translation() - SpeakerTargetOffset;
	}
};

frc::Translation2d 	TargetProvider::GetPassLocation() {
	if (isRedAlliance()) {
		return pathplanner::GeometryUtil::flipFieldPosition(PassLocation);
	} else {
		return PassLocation;
	}
}


