#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/CoralTroughSubsystem.h"

namespace autos {
  enum class AutoProgram {
    kFallback,
    kCenter,
    kOpponentBarge,
    kTeamBarge,
  };

  /**
   * Fallback auto to drive off line if loading the desired auto fails 
   */
  frc2::CommandPtr FallbackAuto(DriveSubsystem& driveSubsystem);

  /**
   * Auto starting in the center lane and dropping a single coral on L1
   */
  frc2::CommandPtr CenterAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem);

  /**
   * Auto starting in the processor side lane (36in from the wall) and dropping a single coral on L1
   */
  frc2::CommandPtr ProcessorSideAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem);

  /**
   * Auto starting in the barge side lane (36in from wall) and dropping a single coral on L1
   */
  frc2::CommandPtr BargeSideAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem);
}  // namespace autos
