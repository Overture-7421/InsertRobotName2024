// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include "OvertureLib/Math/Utils.h"
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/Shooter/Shooter.h"

#include "main/Commands/UtilityFunctions/UtilityFunctions.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class VisionSpeakerCommand
    : public frc2::CommandHelper<frc2::Command, VisionSpeakerCommand> {
 public:
  VisionSpeakerCommand(Chassis* chassis, SuperStructure* SuperStructure, Shooter* shooter, frc::XboxController* joystick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  InterpolatingTable<units::meter_t, double> distanceToUpperAngleTable{
    {1.3_m, -35.00},
    {1.5_m, -35.00},
    {1.7_m, -32.00},
    {1.9_m, -28.00},
    {2.1_m, -25.00},
    {2.3_m, -23.00},
    {2.5_m, -16.00},
    {2.7_m, -16.00},
    {2.9_m, -12.00},
    {3.1_m, -10.00},
    {3.3_m, -8.00},
    {3.5_m, -15.00},
    {3.7_m, -11.00},    
    {3.9_m, -14.00},
    {4.1_m, -17.00},
    {4.3_m, -16.00},
    {4.5_m, -16.00}    
    };

  InterpolatingTable<units::meter_t, double> distanceToLowerAngleTable{
    {1.3_m, -10.0},
    {1.5_m, -10.0},
    {1.7_m, -10.0},
    {1.9_m, -10.0},
    {2.1_m, -10.0},
    {2.3_m, -15.0},
    {2.5_m, -20.0},
    {2.7_m, -20.0},
    {2.9_m, -20.0},
    {3.1_m, -20.0},
    {3.3_m, -20.0},
    {3.5_m, -15.0},
    {3.7_m, -15.0},    
    {3.9_m, -15.0},
    {4.1_m, -10.0},
    {4.3_m, -10.0},
    {4.5_m, -10.0}   
    };

  InterpolatingTable<units::meter_t, double> distanceToVelocityTable{
    {1.3_m, 80.0},
    {1.5_m, 80.0},
    {1.7_m, 80.0},
    {1.9_m, 80.0},
    {2.1_m, 80.0},
    {2.3_m, 90.0},
    {2.5_m, 90.0},
    {2.7_m, 90.0},
    {2.9_m, 90.0},
    {3.1_m, 90.0},
    {3.3_m, 95.0},
    {3.5_m, 95.0},
    {3.7_m, 95.0},    
    {3.9_m, 95.0},
    {4.1_m, 100.0},
    {4.3_m, 100.0},
    {4.5_m, 110.0}   
    };

  SuperStructure* superStructure;
  Chassis* chassis;
  Shooter* shooter;
  frc::XboxController* joystick;

  TargetingWhileMoving dynamicTarget { 
    {
    {1.3_m, 0.1_s},
    {1.5_m, 0.1_s},
    {1.7_m, 0.1_s},
    {1.9_m, 0.1_s},
    {2.1_m, 0.2_s},
    {2.3_m, 0.2_s},
    {2.5_m, 0.2_s},
    {2.7_m, 0.2_s},
    {2.9_m, 0.2_s},
    {3.1_m, 0.3_s},
    {3.3_m, 0.3_s},
    {3.5_m, 0.3_s},
    {3.7_m, 0.3_s},    
    {3.9_m, 0.4_s},
    {4.1_m, 0.4_s},
    {4.3_m, 0.4_s},
    {4.5_m, 0.4_s}   
    }
  };
  units::meter_t distance = 0.0_m;
  frc::Rotation2d angle;
};
