#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/CoralTroughSubsystem.h"

namespace autos {
  /**
   * Example static factory for a path-based autonomous command.
   */
  frc2::CommandPtr CenterAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem);
}  // namespace autos
