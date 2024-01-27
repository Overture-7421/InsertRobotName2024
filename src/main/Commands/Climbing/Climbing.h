#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"

#include <vector>
#include <utility>

enum class ClimbingLocation {
	Left,
	Right,
	Back
};

std::ostream& operator<< (std::ostream& os, ClimbingLocation location);

const std::vector<std::pair<ClimbingLocation, frc::Pose2d>> climbingLocations{
	{ClimbingLocation::Left, {{4.52_m, 4.67_m}, {120_deg}}},
	{ClimbingLocation::Right,{{4.57_m, 3.51_m}, {-120_deg}}},
	{ClimbingLocation::Back, {{5.51_m, 4.10_m}, {0_deg}}}
};

frc2::CommandPtr Climb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms);

