#pragma once

#include <units/length.h>
#include <frc/geometry/Translation2d.h>

namespace ChassisConstants {
    const static double RotationGearRatio = 150.0 / 7.0;
    const static double DriveGearRatio =  5.9027777;
    const static units::meter_t WheelDiameter = 0.1016_m;

    const static frc::Translation2d FrontLeftModuleTranslation = {10.39_in, 10.39_in};
    const static frc::Translation2d FrontRightModuleTranslation = {10.39_in, -10.39_in};
     const static frc::Translation2d BackLeftModuleTranslation = {-10.39_in, 10.39_in};
    const static frc::Translation2d BackRightModuleTranslation = {-10.39_in, -10.39_in};
};