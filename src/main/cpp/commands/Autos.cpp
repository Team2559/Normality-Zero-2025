#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "commands/Autos.h"

frc2::CommandPtr autos::CenterAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem) {
  return frc2::FunctionalCommand(
    [&]() -> void {
      driveSubsystem->ResetDrive();
    },
    [&]() -> void {
      driveSubsystem->Drive(-0.3_mps, 0.0_mps, 0.0_rad_per_s, true);
    },
    [&](bool wasCancelled) -> void {
      // TODO: Maybe make a stop function lol
      driveSubsystem->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true);
    },
    [&]() -> bool {
      return units::math::abs(driveSubsystem->GetPose().X()) >= 1.5_m;
    },
    {driveSubsystem}
  ).AndThen(
    coralTroughSubsystem->DispenseCoral()
  );
}
