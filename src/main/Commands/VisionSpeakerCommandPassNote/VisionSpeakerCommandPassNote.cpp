// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommandPassNote.h"

#include <frc/MathUtil.h>

VisionSpeakerCommandPassNote::VisionSpeakerCommandPassNote(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, TargetProvider* targetProvider, Storage* storage, PassNote upOrDown) : headingHelper({ 11.0, 0.5, 0.6, {13_rad_per_s, 18_rad_per_s_sq * 2} }, chassis) {
	AddRequirements({ superStructure, shooter, storage });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->storage = storage;
	this->targetProvider = targetProvider;
	this->upOrDown = upOrDown;
}

// Called when the command is initially scheduled.
void VisionSpeakerCommandPassNote::Initialize() {
	chassis->enableSpeedHelper(&headingHelper);
	passLocation = targetProvider->GetPassLocation();

	if (upOrDown == PassNote::High) {
		targetState = SuperStructureConstants::HighPassingState;
		targetShooterVelocity = 70.0;
	} else {
		targetState = SuperStructureConstants::LowPassingState;
		targetShooterVelocity = 120.0;
	}

	Timer.Reset();
	Timer.Stop();
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommandPassNote::Execute() {
	frc::Pose2d chassisPose = chassis->getEstimatedPose();

	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = passLocation - chassisLoc;
	distance = chassisToTarget.Distance({ 0_m, 0_m });
	angle = chassisToTarget.Angle().RotateBy({ 180_deg });

	headingHelper.setTargetAngle(angle.Radians());
	units::degree_t targetLowerAngle = targetState.lowerAngle;
	units::degree_t targetUpperAngle = targetState.upperAngle;
	superStructure->setTargetCoord({ targetLowerAngle, targetUpperAngle });
	shooter->setTargetVelocity(targetShooterVelocity);

	units::degree_t headingTolerance = 2_deg + units::degree_t(std::clamp(1 - distance.value() / 6.0, 0.0, 1.0) * 8.0); // Heading tolerance extra of X deg when close, more precise when further back;
	units::degree_t headingError = units::math::abs(frc::InputModulus(angle.Degrees() - chassisPose.Rotation().Degrees(), -180_deg, 180_deg));

	bool lowerAngleInTolerance = std::abs((targetLowerAngle - superStructure->getLowerAngle()).value()) < 1.0;
	bool upperAngleInTolerance = std::abs((targetUpperAngle - superStructure->getUpperAngle()).value()) < 2.0;
	bool headingInTolerance = headingError < headingTolerance;
	bool shooterSpeedInTolerance = (targetShooterVelocity - 5.0) < shooter->getCurrentVelocity();

	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/LowerAngleReached", lowerAngleInTolerance);
	frc::SmartDashboard::PutNumber("VisionSpeakerCommand/Distance", distance.value());
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/UpperAngleReached", upperAngleInTolerance);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/HeadingReached", headingInTolerance);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/ShooterReached", shooterSpeedInTolerance);

	if (lowerAngleInTolerance && upperAngleInTolerance && headingInTolerance && shooterSpeedInTolerance) {
		Timer.Start();
		storage->setVoltage(StorageConstants::ScoreVolts);
	}
}

// Called once the command ends or is interrupted.
void VisionSpeakerCommandPassNote::End(bool interrupted) {
	chassis->disableSpeedHelper();
	storage->storageCommand(StorageConstants::StopVolts);
}

// Returns true when the command should end.
bool VisionSpeakerCommandPassNote::IsFinished() {
	if (Timer.Get() > 0.2_s) {
		return true;
	}

	return false;
}