#include "AlignToTrackedObject.h"

#include <frc/controller/ProfiledPIDController.h>
#include <units/angular_acceleration.h>

frc2::CommandPtr AlignToTrackedObject(Chassis* chassis, photon::PhotonCamera* camera) {

    frc::ProfiledPIDController<units::degree_t> alignController {0.0, 0.0, 0.0, {0_deg_per_s, 0_deg_per_s_sq}};

    return frc2::cmd::RunOnce([chassis] {
        chassis->setVyOverride(true);
    }).AndThen(frc2::cmd::Run([=] mutable {

       const auto result = camera->GetLatestResult();
       if(!result.HasTargets()){
        return;
       }

      const auto target = result.GetBestTarget();      
      double targetVy = alignController.Calculate(units::degree_t(target.GetYaw()), units::degree_t(0)); 
    
      chassis->setVyTarget(units::meters_per_second_t(targetVy));
    })).FinallyDo([=] {
        chassis->setVyOverride(false);
    });
}