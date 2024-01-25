#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include "main/Subsystems/Chassis/Chassis.h"
#include <vector>
#include <utility>

enum class ClimbingLocation { 
    Left,
    Right,
    Back
};

std::ostream& operator<< (std::ostream& os, ClimbingLocation location);

const std::vector<std::pair<ClimbingLocation, frc::Translation2d>> climbingLocations {
    {ClimbingLocation::Left, {4.30_m, 5.08_m}},
    {ClimbingLocation::Right,{4.30_m, 3.02_m}},
    {ClimbingLocation::Back, {5.89_m, 4.10_m}}
};

frc2::CommandPtr Climb(Chassis* chassis);

