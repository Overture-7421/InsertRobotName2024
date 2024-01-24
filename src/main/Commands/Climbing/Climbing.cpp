#include "Climbing.h"

frc2::CommandPtr Climb() {
	auto path = pathplanner::PathPlannerPath::fromPathFile("AMP Climb");

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		3.0_mps, 4.0_mps_sq,
		540_deg_per_s, 720_deg_per_s_sq);

	return pathplanner::AutoBuilder::pathfindThenFollowPath(
		path,
		constraints
	);
};
