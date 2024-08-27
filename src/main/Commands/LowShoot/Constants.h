#pragma once

#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include <OvertureLib/Math/InterpolatingTable/InterpolatingTable.h>

namespace LowShootConstants {
	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToLowerAngleTable{
		{1.66_m, -31.5_deg},
		{1.9_m, -31.5_deg},
		{2.4_m, -31.5_deg},
		{2.9_m, -31.5_deg},
		{3.4_m, -31.5_deg},
		{2.4_m, -31.5_deg},
		{3.9_m, -31.5_deg},
		{4.4_m, -31.5_deg},
		{4.9_m, -31.5_deg},
		{5.4_m, -31.5_deg},
		{5.9_m, -31.5_deg},
		{6.4_m, -31.5_deg},
		{6.9_m, -31.5_deg}

	};

	// Red Alliance
	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToUpperAngleTable{
		{1.66_m, 67_deg},
		{1.9_m, 86_deg},
		{2.4_m, 86_deg},
		{2.9_m, 87_deg},
		{3.4_m, 87_deg},
		{3.9_m, 87_deg},
		{4.4_m, 87_deg},
		{4.9_m, 87_deg},
		{5.4_m, 89_deg},
		{5.9_m, 89_deg},
		{6.4_m, 89.2_deg},
		{6.9_m, 89.7_deg}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToVelocityTable{
		{1.30_m, 70.0},
		{1.66_m, 70.0},
		{1.9_m, 70.0},
		{2.4_m, 70.0},
		{2.9_m, 70.0},
		{3.4_m, 70.0},
		{3.9_m, 70.0},
		{4.4_m, 70.0},
		{4.9_m, 70.0},
		{5.4_m, 70.0},
		{5.9_m, 70.0},
		{6.4_m, 70.0},
		{6.9_m, 60.0}


	};

	static const double multiplier = 1;

	static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
	  {1.66_m, 0.22_s * multiplier},
	  {1.9_m, 0.22_s * multiplier},
	  {2.4_m, 0.25_s * multiplier},
	  {2.9_m, 0.23_s * multiplier},
	  {3.4_m, 0.26_s * multiplier},
	  {3.9_m, 0.28_s * multiplier},
	  {4.4_m, 0.34_s * multiplier},
	  {4.9_m, 0.37_s * multiplier},
	  {5.4_m, 0.38_s * multiplier},
	  {5.9_m, 0.43_s * multiplier},
	  {6.4_m, 0.48_s * multiplier},
	  {6.9_m, 0.51_s * multiplier}


	};
};