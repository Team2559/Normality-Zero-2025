#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "commands/Autos.h"

constexpr units::meters_per_second_t kDriveSpeed = 0.5_mps;
constexpr units::degrees_per_second_t kTurnSpeed = 30_deg_per_s;

frc2::CommandPtr autos::CenterAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem) {
  return frc2::FunctionalCommand(
    [&]() -> void {
      driveSubsystem->ResetDrive();
    },
    [&]() -> void {
      driveSubsystem->Drive(-kDriveSpeed, 0.0_mps, 0.0_rad_per_s, true);
    },
    [&](bool wasCancelled) -> void {
      driveSubsystem->Stop();
    },
    [&]() -> bool {
      return units::math::abs(driveSubsystem->GetPose().X()) >= 1.9_m;
    },
    {driveSubsystem}
  ).AndThen(
    coralTroughSubsystem->DispenseCoral()
  );
}

frc2::CommandPtr autos::SideAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem, Side side) {
  return frc2::FunctionalCommand(
    [&]() -> void {
      driveSubsystem->ResetDrive();
    },
    [&]() -> void {
      driveSubsystem->Drive(-kDriveSpeed, 0.0_mps, 0.0_rad_per_s, true);
    },
    [&](bool wasCancelled) -> void {
      driveSubsystem->Stop();
    },
    [&]() -> bool {
      return units::math::abs(driveSubsystem->GetPose().X()) >= 1.16_m;
    },
    {driveSubsystem}
  ).AndThen(
    frc2::FunctionalCommand(
      [&]() -> void {
      },
      [&]() -> void {
        units::degrees_per_second_t turnSpeed;
        switch (side) {
          case Side::kLeft:
            turnSpeed = kTurnSpeed;
            break;
          case Side::kRight:
            turnSpeed = -kTurnSpeed;
            break;
        }
        driveSubsystem->Drive(0.0_mps, 0.0_mps, turnSpeed, true);
      },
      [&](bool wasCancelled) -> void {
        driveSubsystem->Stop();
      },
      [&]() -> bool {
        return units::math::abs(driveSubsystem->GetPose().ToPose2d().Rotation().Degrees()) >= 60_deg;
      },
      {driveSubsystem}
    ).ToPtr()
  ).AndThen(
    frc2::FunctionalCommand(
      [&]() -> void {
        driveSubsystem->ResetDrive();
      },
      [&]() -> void {
        driveSubsystem->Drive(-kDriveSpeed, 0.0_mps, 0.0_rad_per_s, true);
      },
      [&](bool wasCancelled) -> void {
        driveSubsystem->Stop();
      },
      [&]() -> bool {
        frc::Pose3d pose = driveSubsystem->GetPose();
        // Compare in square units to avoid expensive sqrt operation
        return units::math::pow<2>(pose.X()) + units::math::pow<2>(pose.Y()) >= units::math::pow<2>(1.6_m);
      },
      {driveSubsystem}
    ).ToPtr()
  ).AndThen(
    coralTroughSubsystem->DispenseCoral()
  );
}

frc2::CommandPtr autos::LeftAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem) {
  return SideAuto(driveSubsystem, coralTroughSubsystem, Side::kLeft);
}

frc2::CommandPtr autos::RightAuto(DriveSubsystem* driveSubsystem, CoralTroughSubsystem* coralTroughSubsystem) {
  return SideAuto(driveSubsystem, coralTroughSubsystem, Side::kRight);
}
