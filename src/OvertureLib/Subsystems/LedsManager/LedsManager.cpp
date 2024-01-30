// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LedsManager.h"

LedsManager::LedsManager(int pwmPort, int ledLength) : ledStrip(pwmPort){
    ledBuffer.reserve(ledLength);

    ledStrip.SetLength(ledLength);
    ledStrip.SetData(ledBuffer);
    ledStrip.Start();
};

// This method will be called once per scheduler run
void LedsManager::Periodic() {
    UpdateLeds(ledBuffer);
    ledStrip.SetData(ledBuffer);
}
