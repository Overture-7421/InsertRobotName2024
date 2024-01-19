// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "main/Subsystems/Storage/Storage.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveStorage
    : public frc2::CommandHelper<frc2::Command, MoveStorage> {
 public:
  MoveStorage(Storage* m_Storage, units::volt_t m_voltage);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Storage* m_Storage;
  units::volt_t m_voltage;
};
