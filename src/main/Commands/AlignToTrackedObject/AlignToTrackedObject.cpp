#include "AlignToTrackedObject.h"

#include <frc/controller/ProfiledPIDController.h>
#include <units/angular_acceleration.h>

frc2::CommandPtr AlignToTrackedObject(Chassis* chassis, photon::PhotonCamera* camera) {

    frc::ProfiledPIDController<units::degree> alignController {0.0444, 0.0, 0.0, {120_deg_per_s, 360_deg_per_s_sq}, RobotConstants::LoopTime};

    return frc2::cmd::RunOnce([chassis] {
        chassis->setVyOverride(false);
    }).AndThen(frc2::cmd::Run([=]() mutable{

       const auto result = camera->GetLatestResult();
       if(!result.HasTargets()){
        chassis->setVyOverride(false);
        chassis->setVyTarget(units::meters_per_second_t(0));
        return;
       }
        chassis->setVyOverride(true);

        const auto target = result.GetBestTarget();      
        double targetVy = alignController.Calculate(units::degree_t(target.GetYaw()), units::degree_t(0)); 
    
        chassis->setVyTarget(units::meters_per_second_t(targetVy));
    })).FinallyDo([=] {
        chassis->setVyOverride(false);
        chassis->setVyTarget(units::meters_per_second_t(0));
    });
}