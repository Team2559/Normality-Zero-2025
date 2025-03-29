#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"

namespace autos {
  /**
   * Example static factory for a path-based autonomous command.
   */
  frc2::CommandPtr ExampleAuto(DriveSubsystem& subsystem);
}  // namespace autos
