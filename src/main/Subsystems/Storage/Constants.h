#pragma once
#include <units/voltage.h>

namespace StorageConstants {
    const static units::volt_t GroundGrabVolts = 3_V;
    const static units::volt_t SourceGrabVolts = -3_V;
    const static units::volt_t AmpScoreVolts = -3_V;
};