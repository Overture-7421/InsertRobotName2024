#pragma once
#include "SuperStructureState.h"

namespace SuperStructureConstants {
    const static double LowerAngleGearRatio = 230.4;
    const static double UpperAngleGearRatio = 90.0;

    const static double LowerAngleLowerLimit = -29.8;
    const static double LowerAngleUpperLimit = 90;

    const static double UpperAngleLowerLimit = -130;
    const static double UpperAngleUpperLimit = 0;

    const static double LowerAngleSafetyThreshold = -25;
    const static double UpperAngleSafetyLimit = 0;

    const static SuperStructureState GroundGrabState = {-LowerAngleLowerLimit, 0};
    const static SuperStructureState SourceGrabState = { -LowerAngleLowerLimit, 0.0 };
    const static SuperStructureState ManualSpeakerState = {-12.0, -35.0};
    const static SuperStructureState AmpState = { 65.0, -30.0 };
};