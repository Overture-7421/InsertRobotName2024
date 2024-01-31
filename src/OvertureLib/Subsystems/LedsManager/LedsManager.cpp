// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LedsManager.h"
#include <iostream>

LedsManager::LedsManager(int pwmPort, int ledLength, const std::map<LedStripName, LedStripRange>& ledStripMap) : ledStrip(pwmPort), ledStripMap(ledStripMap){

    for(auto ledStrip : ledStripMap) {
        if(ledStrip.second.endLed <= ledStrip.second.startLed) {
            throw std::logic_error("Led strip has an end led that is before the start led!!!");
        }

        if(ledStrip.second.startLed >= ledLength || ledStrip.second.endLed >= ledLength){
            throw std::logic_error("Led strip has a start or end led that is greater than the total length!!!");
        }
    }


    ledBuffer.resize(ledLength);

    ledStrip.SetLength(ledLength);
    ledStrip.SetData(ledBuffer);
    ledStrip.Start();
};

void LedsManager::setLedStrip(LedStripName name, const LedStrip& stripData) {
    if(!ledStripMap.contains(name)) {
        return;
    }

    const auto ledStrip = ledStripMap.at(name);

    if(stripData.size() >= ledStrip.endLed - ledStrip.startLed){
        std::cout << "Warning: Tried to set led strip " << name << " to a vector that is bigger than the strip!" << std::endl;
        return;
    }

    if(ledStrip.reversed) {
        std::copy(stripData.rbegin(), stripData.rend(), ledBuffer.begin() + ledStrip.startLed);
    }else{
        std::copy(stripData.begin(), stripData.end(), ledBuffer.begin() + ledStrip.startLed);
    }
}

void LedsManager::setLedStripAll(const LedStrip& stripData){
    if(stripData.size() > ledBuffer.size()){
        std::cout << "Warning: Tried to set all the leds to a vector that is bigger than the strip!" << std::endl;
        return;
    }

    std::copy(stripData.begin(), stripData.end(), ledBuffer.begin());
}

const frc::AddressableLED& LedsManager::getLedStripAll() {
    return ledStrip;
}

const LedStrip& LedsManager::getLedStripState(LedStripName name){
    if(!ledStripMap.contains(name)) {
        return {};
    }

    const auto ledStrip = ledStripMap.at(name);
    return {ledBuffer.begin() + ledStrip.startLed, ledBuffer.begin() + ledStrip.endLed};
}

// This method will be called once per scheduler run
void LedsManager::Periodic() {
    ledStrip.SetData(ledBuffer);
}
