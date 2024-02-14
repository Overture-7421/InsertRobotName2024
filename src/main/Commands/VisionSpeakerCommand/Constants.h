#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"

namespace VisionSpeakerCommandConstants {
	static const InterpolatingTable<units::meter_t, double> DistanceToLowerAngleTable{
		{1.4_m, -10.0},
		{1.9_m, -10.0},
		{2.4_m, -10.0},
		{2.9_m, -10.0},
		{3.4_m, -10.0},
		{3.9_m, -10.0}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToUpperAngleTable{
		{1.4_m, -30.0},
		{1.9_m, -25.0},
		{2.4_m, -22.0},
		{2.9_m, -18.0},
		{3.4_m, -15.0},
		{3.9_m, -13.0}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToVelocityTable{
		{1.4_m, 100.0},
		{1.9_m, 100.0},
		{2.4_m, 100.0},
		{2.9_m, 100.0},
		{3.4_m, 110.0},
		{3.9_m, 120.0}
	};

    static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
      {1.4_m, 0.20_s}, 
      {1.9_m, 0.25_s}, 
      {2.4_m, 0.30_s}, 
      {2.9_m, 0.40_s}, 
      {3.4_m, 0.33_s}, 
      {3.9_m, 0.33_s}, 
    };

    static const frc::Translation2d TargetLocation = { 0.06_m, 5.54_m };
};