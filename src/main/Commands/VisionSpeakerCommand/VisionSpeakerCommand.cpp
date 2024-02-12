// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommand.h"

#include <frc/MathUtil.h>

VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, frc::XboxController* joystick) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->joystick = joystick;
}

VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, Storage* storage) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter, storage });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->storage = storage;
}

// Called when the command is initially scheduled.
void VisionSpeakerCommand::Initialize() {
  chassis->setHeadingOverride(true);

  if(shouldFlip()){
    dynamicTarget.setTargetLocation(pathplanner::GeometryUtil::flipFieldPosition({0.06_m, 5.54_m}));
  }else{
    dynamicTarget.setTargetLocation({0.06_m, 5.54_m});
  }

	Timer.Reset();
	Timer.Stop();
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommand::Execute() {
	frc::Pose2d chassisPose = chassis->getOdometry();
	frc::Translation2d speakerLoc = dynamicTarget.getMovingTarget(chassisPose, chassis->getFieldRelativeSpeeds(), chassis->getFIeldRelativeAccels());
	field.SetRobotPose({speakerLoc, {0_deg}});
	
	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = speakerLoc - chassisLoc;
	distance = chassisToTarget.Distance({ 0_m, 0_m });
	angle = chassisToTarget.Angle().RotateBy({ 180_deg });
	frc::SmartDashboard::PutNumber("Distance to Target", double(distance));
	frc::SmartDashboard::PutNumber("Angle to Target", angle.Degrees().value());

	chassis->setTargetHeading(angle);
	double targetLowerAngle = distanceToLowerAngleTable[distance];
	double targetUpperAngle = distanceToUpperAngleTable[distance];
	double targetShooterVelocity = distanceToVelocityTable[distance];
	superStructure->setTargetCoord({ targetLowerAngle, targetUpperAngle });
	shooter->setVelocityVoltage(targetShooterVelocity);

	units::degree_t headingTolerance = 2_deg + units::degree_t(std::clamp( 1 - distance.value() / 8.0, 0.0, 1.0) * 10); // Heading tolerance extra of X deg when close, more precise when further back;
	units::degree_t headingError = frc::InputModulus(angle.Degrees() - chassisPose.Rotation().Degrees(), -180_deg, 180_deg);

	bool lowerAngleInTolerance = std::abs(targetLowerAngle - superStructure->getLowerAngle()) < (2.00);
	bool upperAngleInTolerance = std::abs(targetUpperAngle - superStructure->getUpperAngle()) < (2.00);
	bool headingInTolerance = units::math::abs(headingError) < headingTolerance;
	bool shooterSpeedInTolerance = std::abs(targetShooterVelocity - shooter->getCurrentVelocity()) < 3.00;

	frc::SmartDashboard::PutBoolean("lowerAngleInTolerance ", lowerAngleInTolerance);
	frc::SmartDashboard::PutBoolean("upperAngleInTolerance ", upperAngleInTolerance);
	frc::SmartDashboard::PutNumber("headingDynamicTolerance ", headingTolerance.value());
	frc::SmartDashboard::PutBoolean("headingInTolerance ", headingInTolerance);
	frc::SmartDashboard::PutNumber("headingError", headingError.value());
	frc::SmartDashboard::PutBoolean("shooterSpeedInTolerance ", shooterSpeedInTolerance);

	if (lowerAngleInTolerance && upperAngleInTolerance && headingInTolerance && shooterSpeedInTolerance) {
		if (joystick == nullptr) {
			Timer.Start();
			storage->setVoltage(3_V);
		} else {
			joystick->SetRumble(frc::GenericHID::kBothRumble, 1.0);
		}
	} else {
		if (joystick != nullptr) {
			joystick->SetRumble(frc::GenericHID::kBothRumble, 0.0);
		}
	}
}

// Called once the command ends or is interrupted.
void VisionSpeakerCommand::End(bool interrupted) {
	chassis->setHeadingOverride(false);
	if (joystick == nullptr) {
		storage->setVoltage(0_V);
	} else {
		joystick->SetRumble(frc::GenericHID::kBothRumble, 0.0);
	}
}

// Returns true when the command should end.
bool VisionSpeakerCommand::IsFinished() {
	if (joystick == nullptr && Timer.Get() >= 0.1_s) {
		return true;
	}

	return false;
}
