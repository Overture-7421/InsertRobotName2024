#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"

namespace VisionSpeakerCommandConstants {
	static const InterpolatingTable<units::meter_t, double> DistanceToUpperAngleTable{
	  {0_m, -1.00},
	  {1_m, -5.00},
	  {2_m, -15.00},
	  {3_m, -20.00},
	  {4_m, -25.00},
	  {5_m, -30.00},
	  {6_m, -35.00}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToLowerAngleTable{
	  {1_m, 1.00},
	  {2_m, 3.00},
	  {3_m, 5.00},
	  {4_m, 7.00},
	  {5_m, 9.00},
	  {6_m, 11.00},
	  {7_m, 13.00}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToVelocityTable{
	  {0.1_m, 1.00},
	  {0.2_m, 2.00},
	  {0.3_m, 3.00},
	  {0.4_m, 4.00},
	  {0.5_m, 5.00},
	  {0.6_m, 6.00},
	  {0.7_m, 7.00}
	};

    static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
	  {1_m, 0.2_s},
	  {2_m, 0.3_s},
	  {3_m, 0.4_s},
	  {4_m, 0.5_s},
	  {5_m, 0.6_s},
	  {6_m, 0.7_s},
	  {7_m, 0.8_s}
    };

    static const frc::Translation2d TargetLocation = { 0.06_m, 5.54_m };
};