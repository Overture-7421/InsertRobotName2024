#pragma once
#include "SuperStructureState.h"

namespace SuperStructureConstants {
	const static double LowerAngleGearRatio = 172.8;
	const static double UpperAngleGearRatio = 90.0;

	const static double LowerAngleLowerLimit = -31;
	const static double LowerAngleUpperLimit = 90;

	const static double UpperAngleLowerLimit = -130;
	const static double UpperAngleUpperLimit = -0.5;

	const static double LowerAngleSafetyThreshold = -25;
	const static double UpperAngleSafetyLimit = -10;

	const static SuperStructureState GroundGrabState = { LowerAngleLowerLimit, -5.0 };
	const static SuperStructureState SourceGrabState = { LowerAngleLowerLimit, -0.35 };
	const static SuperStructureState ManualSpeakerState = { -12.0, -35.0 };
	const static SuperStructureState AmpState = { 65.0, -30.0 };
	const static SuperStructureState SmoothCloseState = { -26.0, -7.0 };
	const static SuperStructureState HighPassingState = { -12.0, -38.0 };
	const static SuperStructureState LowPassingState = { -4.0, -5.0 };
};