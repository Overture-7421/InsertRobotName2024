#pragma once
#include "SuperStructureState.h"

namespace SuperStructureConstants {
    const static double LowerAngleGearRatio = 230.0;
    const static double UpperAngleGearRatio = 90.0;

    const static double LowerAngleLowerLimit = -25;
    const static double LowerAngleUpperLimit = 90;

    const static double UpperAngleLowerLimit = -110;
    const static double UpperAngleUpperLimit = 0;

    const static double LowerAngleSafetyThreshold = -25;
    const static double UpperAngleSafetyLimit = -10;

    const static SuperStructureState GroundGrabState = {-27.3,-13.3};
    const static SuperStructureState SourceGrabState = { 70.0, -50.0 };
    const static SuperStructureState ManualSpeakerState = {-20.0, -17.0};
    const static SuperStructureState AmpState = { 65.0, -30.0 };
};