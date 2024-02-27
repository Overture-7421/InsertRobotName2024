#pragma once

#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"

namespace VisionSpeakerCommandConstants {
	static const InterpolatingTable<units::meter_t, double> DistanceToLowerAngleTable{
		{1.4_m, -15.0},
		{1.9_m, -15.0},
		{2.4_m, -15.0},
		{2.9_m, -15.0},
		{3.4_m, -15.0},
		{3.9_m, -15.0},
		{4.4_m, -15.0},
		{4.9_m, -15.0},
		{5.4_m, -15.0},
		{5.9_m, -15.0},
		{6.4_m, -15.0}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToUpperAngleTable{
		{1.4_m, -35.0},
		{1.9_m, -25.0},
		{2.4_m, -18.5},
		{2.9_m, -15.5},
		{3.4_m, -12.5},
		{3.9_m, -10.5},
		{4.4_m, -9.0},
		{4.9_m, -8.2},
		{5.4_m, -7.2},
		{5.9_m, -6.5},
		{6.4_m, -6.0}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToVelocityTable{
		{1.4_m, 170.0},
		{1.9_m, 170.0},
		{2.4_m, 170.0},
		{2.9_m, 170.0},
		{3.4_m, 170.0},
		{3.9_m, 170.0},
		{4.4_m, 170.0},
		{4.9_m, 170.0},
		{5.4_m, 170.0},
		{5.9_m, 170.0},
		{6.4_m, 170.0}
	};

	static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
	  {1.4_m, 0.18_s},
	  {1.9_m, 0.19_s},
	  {2.4_m, 0.21_s},
	  {2.9_m, 0.26_s},
	  {3.4_m, 0.29_s},
	  {3.9_m, 0.34_s},
	  {4.4_m, 0.38_s},
	  {4.9_m, 0.41_s},
	  {5.4_m, 0.43_s},
	  {5.9_m, 0.48_s},
	  {6.4_m, 0.51_s}
	};

	static const frc::Translation2d TargetLocation = { 0.0_m, 5.55_m };
};