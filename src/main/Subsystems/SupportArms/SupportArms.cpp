// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArms.h"

SupportArms::SupportArms(){

};

void SupportArms::SetAngle(double angle) {
    servo.SetAngle(angle);
};

void SupportArms::Periodic(){

};