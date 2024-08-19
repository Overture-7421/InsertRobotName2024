#pragma once

#include <OvertureLib/Math/InterpolatingTable/InterpolatingTable.h>
#include <frc/geometry/Pose3d.h>
#include <units/length.h>

namespace AllignToNoteConstants {

	static const units::meter_t NoteWidth = 14_in;
	static const units::meter_t CameraHeight = 0.5_m;

	static const frc::Pose3d CameraOffset{ {15.320684_in, 0_in, 9.288683_in}, {75_deg, 0_deg, 0_deg} };

	static const InterpolatingTable<double, double> PixelsToAngle{
		{0.0, 0.0},
		{240.0, 30.0},
		{480.0, 70.0}
	};

	static const double maxOutput = 0.5;
}