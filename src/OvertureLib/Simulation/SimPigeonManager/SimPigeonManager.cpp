// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SimPigeonManager.h"
#include <networktables/NetworkTable.h>
#include <frc/RobotController.h>

SimPigeonManager* SimPigeonManager ::instancePtr = NULL; 


SimPigeonManager::SimPigeonManager(){

}

void SimPigeonManager::SetSimPigeon(OverPigeon* pigeon){
    if(pigeon == NULL || pigeon == 0){
        throw std::invalid_argument("SimPigeonManager given null pointer!");
    }

    this->pigeon = pigeon;
}

void SimPigeonManager::Init(std::string robotName, std::string imuName){
    std::shared_ptr<nt::NetworkTable> ntable = ntInst.GetTable(robotName)->GetSubTable(imuName);
    rollEntry = ntable->GetEntry("roll");
    pitchEntry = ntable->GetEntry("pitch");
    yawEntry = ntable->GetEntry("yaw");

    pigeonSimState = &pigeon->GetSimState();
}

void SimPigeonManager::Update(){
    pigeonSimState->SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    pigeonSimState->SetRoll(units::angle::degree_t(rollEntry.GetDouble(0)));
    pigeonSimState->SetPitch(units::angle::degree_t(pitchEntry.GetDouble(0)));
    pigeonSimState->SetRawYaw(units::angle::degree_t(yawEntry.GetDouble(0)));
    
    ntInst.Flush();
}


