// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ChassisAccelToStructureFF.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

ChassisAccelToStructureFF::ChassisAccelToStructureFF(Chassis* chassis, SuperStructure* superStructure) 
    : chassis(chassis), superStructure(superStructure){

    frc::SmartDashboard::PutNumber("ChassisAccelToStructureFF/MaxAccel", maxAccel.value());
    frc::SmartDashboard::PutNumber("ChassisAccelToStructureFF/AccelToVoltsFactor", accelToVoltsFactor);
}

void ChassisAccelToStructureFF::Periodic() {
    filteredAccel = accelXFilter.Calculate(chassis->getCurrentAccels().ax);
    double clampedAccel = std::clamp(filteredAccel.value(), -maxAccel.value(), maxAccel.value());
    outFF = units::volt_t(accelToVoltsFactor * clampedAccel);
    superStructure->setArbitraryFeedForwardUpper(outFF);
}

void ChassisAccelToStructureFF::shuffleboardPeriodic() {
    maxAccel = units::meters_per_second_squared_t(frc::SmartDashboard::GetNumber("ChassisAccelToStructureFF/MaxAccel", 0.0));
    accelToVoltsFactor = frc::SmartDashboard::GetNumber("ChassisAccelToStructureFF/AccelToVoltsFactor", 0.0);
    frc::SmartDashboard::PutNumber("ChassisAccelToStructureFF/FilteredAccel", filteredAccel.value());
    frc::SmartDashboard::PutNumber("ChassisAccelToStructureFF/AppliedFF", outFF.value());
}