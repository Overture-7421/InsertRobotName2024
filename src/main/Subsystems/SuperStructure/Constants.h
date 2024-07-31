#pragma once
#include "SuperStructureState.h"

namespace SuperStructureConstants {
	const static double LowerAngleGearRatio = 172.8;
	const static double UpperAngleGearRatio = 144.0;

	const static double LowerAngleLowerLimit = -32;
	const static double LowerAngleUpperLimit = 90;

	const static double UpperAngleLowerLimit = -130;
	const static double UpperAngleUpperLimit = -0.2;

	const static double LowerAngleSafetyThreshold = -30;
	const static double UpperAngleSafetyLimit = -23;

	const static SuperStructureState GroundGrabState = { LowerAngleLowerLimit, -22.0 };
	const static SuperStructureState ClosedState = { LowerAngleLowerLimit, UpperAngleUpperLimit };
	const static SuperStructureState ClimbEndState = { 0, UpperAngleUpperLimit };
	const static SuperStructureState SourceGrabState = { LowerAngleLowerLimit, -0.35 };
	const static SuperStructureState ManualSpeakerState = { -12.0, -35.0 };
	const static SuperStructureState AmpState = { 58.0,  -27.0 };
	const static SuperStructureState SmoothCloseState = { -26.0, -7.0 };
	const static SuperStructureState HighPassingState = { -12.0, -38.0 };
	const static SuperStructureState LowPassingState = { 5.0, -5.0 };
};