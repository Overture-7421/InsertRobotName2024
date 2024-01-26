#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"

#include <vector>
#include <utility>

units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Translation2d targetTranslation);

enum class ClimbingLocation { 
    Left,
    Right,
    Back
};

std::ostream& operator<< (std::ostream& os, ClimbingLocation location);

const std::vector<std::pair<ClimbingLocation, frc::Translation2d>> climbingLocations {
    {ClimbingLocation::Left, {4.46_m, 4.79_m}},
    {ClimbingLocation::Right,{4.46_m, 3.43_m}},
    {ClimbingLocation::Back, {5.68_m, 4.10_m}}
};

frc2::CommandPtr Climb(Chassis* chassis, SuperStructure* superStructure);

