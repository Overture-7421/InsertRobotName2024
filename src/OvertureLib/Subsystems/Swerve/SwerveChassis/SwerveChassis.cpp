// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveChassis.h"

/**
* @brief Builds an object of swerve chassis
*/
SwerveChassis::SwerveChassis() {
	AutoBuilder::configureHolonomic(
		[this]() { return getOdometry(); },
		[this](frc::Pose2d pose) { resetOdometry(pose); },
		[this]() { return getRobotRelativeSpeeds(); },
		[this](frc::ChassisSpeeds speeds) { driveRobotRelative(speeds); },
		HolonomicPathFollowerConfig(
			PIDConstants(5.0, 0.0, 0.0),
			PIDConstants(5.0, 0.0, 0.0),
			5.0_mps,
			0.3732276_m,
			ReplanningConfig()
		),
		[]() {
		// Boolean supplier that controls when the path will be mirrored for the red alliance
		// This will flip the path being followed to the red side of the field.
		// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

		auto alliance = frc::DriverStation::GetAlliance();
		if (alliance) {
			return alliance.value() == frc::DriverStation::Alliance::kRed;
		}
		return false;
	},
		this // Reference to this subsystem to set requirements
	);

	frc::SmartDashboard::PutData("Odometry", &field2d);
}

/**
 * @brief Sets the swerve module positions for the kinematics and odometry.
 *
 * @param positions The positions of the modules relative to the center of the robot.
 */
void SwerveChassis::setModulePositions(std::array<frc::Translation2d, 4>* positions) {
	kinematics = new frc::SwerveDriveKinematics<4>{ *positions };
};

/**
* @brief Sets the swerve modules ratios
*
* @param turnRatio The ratio between the turn motor and the wheel
* @param driveRatio The ratio between the drive motor and the wheel
* @param wheelDiameter The diameter of the wheel
*/
void SwerveChassis::setModulesRatios(double turnRatio, double driveRatio, double wheelDiameter) {
	frontRightModule->setGearRatio(turnRatio, driveRatio);
	frontLeftModule->setGearRatio(turnRatio, driveRatio);
	backRightModule->setGearRatio(turnRatio, driveRatio);
	backLeftModule->setGearRatio(turnRatio, driveRatio);

	frontRightModule->setWheelDiameter(wheelDiameter);
	frontLeftModule->setWheelDiameter(wheelDiameter);
	backRightModule->setWheelDiameter(wheelDiameter);
	backLeftModule->setWheelDiameter(wheelDiameter);
}

/**
 * @brief Sets the swerve modules
 *
 * @param frontLeft The front left module
 * @param frontRight The front right module
 * @param backLeft The back left module
 * @param backRight The back right module
 */
void SwerveChassis::setModules(SwerveModule* frontLeft, SwerveModule* frontRight, SwerveModule* backLeft, SwerveModule* backRight) {
	this->frontLeftModule = frontLeft;
	this->frontRightModule = frontRight;
	this->backLeftModule = backLeft;
	this->backRightModule = backRight;

	pigeon->SetYaw(0_deg);

	odometryPos = new std::array<frc::SwerveModulePosition, 4>{
		frontLeftModule->getPosition(),
			frontRightModule->getPosition(),
			backLeftModule->getPosition(),
			backRightModule->getPosition()
	};

	odometry = new frc::SwerveDrivePoseEstimator<4>{
		*kinematics,
			frc::Rotation2d{},
			*odometryPos,
			frc::Pose2d{}
	};

}

/**
 * @brief Sets each module rotator PID values
 *
 * @param kP The proportional value
 * @param kI The integral value
 * @param kD The derivative value
 */
void SwerveChassis::setRotatorPID(double kP, double kI, double kD) {
	backRightModule->setRotatorPIDValues(kP, kI, kD);
	backLeftModule->setRotatorPIDValues(kP, kI, kD);
	frontRightModule->setRotatorPIDValues(kP, kI, kD);
	frontLeftModule->setRotatorPIDValues(kP, kI, kD);
}

/**
 * @brief Sets each module drive PID values
 *
 * @param kP The proportional value
 * @param kI The integral value
 * @param kD The derivative value
 */
void SwerveChassis::setDrivePID(double kP, double kI, double kD) {
	backRightModule->setDrivePIDValues(kP, kI, kD);
	backLeftModule->setDrivePIDValues(kP, kI, kD);
	frontRightModule->setDrivePIDValues(kP, kI, kD);
	frontLeftModule->setDrivePIDValues(kP, kI, kD);
}

/**
 * @brief Sets the modules feedforward values
 *
 * @param kS The static value
 * @param kV The velocity value
 * @param kA The acceleration value
 */
void SwerveChassis::setFeedForward(units::volt_t kS, units::volt_t kV, units::volt_t kA) {
	backRightModule->setFFConstants(kS, kV, kA);
	backLeftModule->setFFConstants(kS, kV, kA);
	frontRightModule->setFFConstants(kS, kV, kA);
	frontLeftModule->setFFConstants(kS, kV, kA);
}

/**
 * @brief Sets the robot target speed in robot relative
 *
 * @param speeds ChassisSpeeds object
 */
void SwerveChassis::driveRobotRelative(frc::ChassisSpeeds speeds) {
	this->linearX = speeds.vx.value();
	this->linearY = speeds.vy.value();
	this->angular = speeds.omega.value();

	wpi::array<frc::SwerveModuleState, 4> desiredStates = kinematics->ToSwerveModuleStates(speeds);

	kinematics->DesaturateWheelSpeeds(&desiredStates, 5.75_mps);

	setModuleStates(desiredStates);
}

/**
 * @brief Sets the robot target speed in field relative
 *
 * @param speeds ChassisSpeeds object
 */
void SwerveChassis::driveFieldRelative(frc::ChassisSpeeds speeds) {
	frc::ChassisSpeeds chassisSpeeds = frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, getOdometry().Rotation()), 0.02_s);

	driveRobotRelative(chassisSpeeds);
}

/**
 * @brief Returns the robot relative speeds
 *
 * @return ChassisSpeeds object
 */
frc::ChassisSpeeds SwerveChassis::getRobotRelativeSpeeds() {
	return kinematics->ToChassisSpeeds(getModuleStates());
}

/**
 * @brief Returns the robot odometry
 *
 * @return Pose2d object
 */
frc::Pose2d SwerveChassis::getOdometry() {
	return odometry->GetEstimatedPosition();
}

/**
 * @brief Resets the robot odometry
 *
 * @param initPose Pose2d object
 */
void SwerveChassis::resetOdometry(frc::Pose2d initPose) {
	odometry->ResetPosition(pigeon->GetRotation2d(), getModulePosition(), initPose);
}

/**
 * @brief Return the robot kinematics
 *
 * @return SwerveDriveKinematics object
 */
const frc::SwerveDriveKinematics<4>& SwerveChassis::getKinematics() {
	return *kinematics;
}

/**
 * @brief Updates odometry using vision
 */
void SwerveChassis::addVisionMeasurement(frc::Pose2d pose, units::second_t timestamp) {
	odometry->AddVisionMeasurement(pose, timestamp);
}

/**
 * @brief Sets the odometry to desired angle
 *
 * @param angle Desired angle
 */
void SwerveChassis::resetAngle(double angle) {
	frc::Pose2d actualOdometry = getOdometry();
	frc::Pose2d newOdometry{ actualOdometry.X(), actualOdometry.Y(), units::degree_t(angle) };
	resetOdometry(newOdometry);
}

/**
 * @brief Updates module states
 *
 * @param desiredStates SwerveModuleState array
 */
void SwerveChassis::setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
	frontLeftModule->setState(desiredStates[0]);
	frontRightModule->setState(desiredStates[1]);
	backRightModule->setState(desiredStates[2]);
	backLeftModule->setState(desiredStates[3]);

	backRightModule->setVoltages();
	backLeftModule->setVoltages();
	frontLeftModule->setVoltages();
	frontRightModule->setVoltages();
}

/**
 * @brief Returns the module states
 *
 * @return SwerveModuleState array
 */
wpi::array<frc::SwerveModuleState, 4> SwerveChassis::getModuleStates() {
	wpi::array<frc::SwerveModuleState, 4> modulePositions{
		frontLeftModule->getState(),
			frontRightModule->getState(),
			backLeftModule->getState(),
			backRightModule->getState()
	};
	return modulePositions;
}

/**
 * @brief Returns the module positions
 *
 * @return SwerveModulePosition array
 */
wpi::array<frc::SwerveModulePosition, 4> SwerveChassis::getModulePosition() {
	wpi::array<frc::SwerveModulePosition, 4> modulePositions{
		frontLeftModule->getPosition(),
			frontRightModule->getPosition(),
			backLeftModule->getPosition(),
			backRightModule->getPosition()
	};
	return modulePositions;
}

/**
 * @brief Returns the robot pitch
 *
 * @return double
 */
double SwerveChassis::getPitch() {
	return pigeon->GetPitch().GetValue().value();
}

/**
 * @brief Returns the robot yaw
 *
 * @return double
 */
double SwerveChassis::getYaw() {
	return pigeon->GetYaw().GetValue().value();
}

/**
 * @brief Returns the robot roll
 *
 * @return double
 */
double SwerveChassis::getRoll() {
	return pigeon->GetRoll().GetValue().value();
}

/**
 * @brief Runs the SysId Quasisstatic command
*/
frc2::CommandPtr SwerveChassis::SysIdQuadstatic(frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(
		frc2::InstantCommand([this]() { sysIdVoltage(0_V); }).ToPtr(),
		frc2::cmd::Wait(0.5_s),
		m_sysIdRoutine.Quasistatic(direction)
	);
}

/**
 * @brief Runs the SysId Dynamic command
*/
frc2::CommandPtr SwerveChassis::SysIdDinamic(frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(
		frc2::InstantCommand([this]() { sysIdVoltage(0_V); }).ToPtr(),
		frc2::cmd::Wait(0.5_s),
		m_sysIdRoutine.Dynamic(direction)
	);
}

/**
 * @brief Sets the voltage for the SysId command
*/
void SwerveChassis::sysIdVoltage(units::volt_t voltage) {
	frontLeftModule->setRawVoltageSpeed(voltage);
	frontRightModule->setRawVoltageSpeed(voltage);
	backLeftModule->setRawVoltageSpeed(voltage);
	backRightModule->setRawVoltageSpeed(voltage);

}

/**
 * @brief Updates the robot odometry
 */
void SwerveChassis::updateOdometry() {
	odometry->Update(pigeon->GetRotation2d(), getModulePosition());
}

void SwerveChassis::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber("Odometry/LinearX", linearX);
	frc::SmartDashboard::PutNumber("Odometry/LinearY", linearY);
	frc::SmartDashboard::PutNumber("Odometry/Angular", angular);

	auto estimatedPos = getOdometry();

	field2d.SetRobotPose(estimatedPos);

	frc::SmartDashboard::PutNumber("Odometry/X", estimatedPos.X().value());
	frc::SmartDashboard::PutNumber("Odometry/Y", estimatedPos.Y().value());
	frc::SmartDashboard::PutNumber("Odometry/Pigeon", estimatedPos.Rotation().Degrees().value());
}