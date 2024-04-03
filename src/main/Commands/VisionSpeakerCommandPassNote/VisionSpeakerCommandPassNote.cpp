// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommandPassNote.h"

#include <frc/MathUtil.h>

VisionSpeakerCommandPassNote::VisionSpeakerCommandPassNote(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, AprilTagCamera* tagCamera, Storage* storage, PassNote upOrDown) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter, storage });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->storage = storage;
	this->tagCamera = tagCamera;
	this->upOrDown = upOrDown;
}

// Called when the command is initially scheduled.
void VisionSpeakerCommandPassNote::Initialize() {
	chassis->setHeadingOverride(true);

	if (isRedAlliance()) {
		speakerLoc = { 15.32_m, 7.11_m };
	} else {
		speakerLoc = { 1.01_m, 7.11_m };
	}

	if(upOrDown == PassNote::High) {
		targetState = SuperStructureConstants::HighPassingState;
		targetShooterVelocity = 80.0;
	}else{
		targetState = SuperStructureConstants::LowPassingState;
		targetShooterVelocity = 100.0;
	}

	Timer.Reset();
	Timer.Stop();
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommandPassNote::Execute() {
	frc::Pose2d chassisPose = chassis->getOdometry();
	frc::Translation2d speakerLoc = this->speakerLoc;

	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = speakerLoc - chassisLoc;
	distance = chassisToTarget.Distance({ 0_m, 0_m });
	angle = chassisToTarget.Angle().RotateBy({ 180_deg });

	chassis->setTargetHeading(angle);
	double targetLowerAngle = targetState.lowerAngle;
	double targetUpperAngle = targetState.upperAngle;
	superStructure->setTargetCoord({ targetLowerAngle, targetUpperAngle });
	shooter->setTargetVelocity(targetShooterVelocity);

	units::degree_t headingTolerance = 2_deg + units::degree_t(std::clamp(1 - distance.value() / 6.0, 0.0, 1.0) * 8.0); // Heading tolerance extra of X deg when close, more precise when further back;
	units::degree_t headingError = units::math::abs(frc::InputModulus(angle.Degrees() - chassisPose.Rotation().Degrees(), -180_deg, 180_deg));

	bool lowerAngleInTolerance = std::abs(targetLowerAngle - superStructure->getLowerAngle()) < 1.0;
	bool upperAngleInTolerance = std::abs(targetUpperAngle - superStructure->getUpperAngle()) < 1.0;
	bool headingInTolerance = headingError < headingTolerance;
	bool shooterSpeedInTolerance = (targetShooterVelocity - 2.0) < shooter->getCurrentVelocity();

	frc::SmartDashboard::PutBoolean("VisionSpeakerCommandPassNote/LowerAngleReached", lowerAngleInTolerance);
	frc::SmartDashboard::PutNumber("VisionSpeakerCommandPassNote/Distance", distance.value());
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommandPassNote/UpperAngleReached", upperAngleInTolerance);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommandPassNote/HeadingReached", headingInTolerance);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommandPassNote/ShooterReached", shooterSpeedInTolerance);

	if (lowerAngleInTolerance && upperAngleInTolerance && headingInTolerance && shooterSpeedInTolerance) {
		Timer.Start();
		storage->setVoltage(StorageConstants::SpeakerScoreVolts);
	}
}

// Called once the command ends or is interrupted.
void VisionSpeakerCommandPassNote::End(bool interrupted) {
	chassis->setHeadingOverride(false);
	storage->setVoltage(0_V);
}

// Returns true when the command should end.
bool VisionSpeakerCommandPassNote::IsFinished() {
	if (Timer.Get() > 0.2_s) {
		return true;
	}

	return false;
}