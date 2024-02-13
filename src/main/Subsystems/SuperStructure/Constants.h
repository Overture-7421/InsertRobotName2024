#pragma once
#include "SuperStructureState.h"

namespace SuperStructureConstants {
    const static double LowerAngleGearRatio = 230.4;
    const static double UpperAngleGearRatio = 90.0;

    const static double LowerAngleLowerLimit = -29;
    const static double LowerAngleUpperLimit = 90;

    const static double UpperAngleLowerLimit = -130;
    const static double UpperAngleUpperLimit = 0;

    const static double LowerAngleSafetyThreshold = -23;
    const static double UpperAngleSafetyLimit = -15;

    const static SuperStructureState GroundGrabState = {-28, 0};
    const static SuperStructureState SourceGrabState = { 70.0, -50.0 };
    const static SuperStructureState ManualSpeakerState = {-20.0, -17.0};
    const static SuperStructureState AmpState = { 65.0, -30.0 };
};