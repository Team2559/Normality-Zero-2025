// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/CoralTroughSubsystem.h"

using namespace frc2;

namespace autos {
  enum class AutoProgram {
    kCenter,
    kTeamBarge,
    kOpponentBarge,
  };

  enum class Side {
    kLeft,
    kRight,
  };

  CommandPtr CenterAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem);
  CommandPtr SideAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem, Side side);
  CommandPtr LeftAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem);
  CommandPtr RightAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem);
}
