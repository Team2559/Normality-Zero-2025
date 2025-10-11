#include "commands/Autos.h"
#include "commands/SwerveTrajectoryCommand.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

static auto centerTrajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("CenterAuto");
static auto processorSideTrajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ProcessorSideAuto");
static auto bargeSideTrajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("BargeSideAuto");
static auto processorSideLoadTrajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ProcessorSideLoadingAuto");
static auto bargeSideLoadTrajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("BargeSideLoadingAuto");

frc2::CommandPtr autos::FallbackAuto(DriveSubsystem& driveSubsystem) {
  return frc2::RunCommand([&]() {
      driveSubsystem.Drive(-0.25_mps, 0.0_mps, 0.0_rad_per_s, true);
    }, {&driveSubsystem}).Until([&]() -> bool {
      return units::math::abs(driveSubsystem.GetPose().X()) >= 1.0_m;
    }).BeforeStarting([]() {
      printf(">>>Running traditional auto\n");
    });
}

frc2::CommandPtr autos::CenterAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem) {
  if (centerTrajectory.has_value()) {
    return SwerveTrajectoryCommand(driveSubsystem, centerTrajectory.value())
      .AndThen(
        coralTroughSubsystem.DispenseCoral()
      )
      .BeforeStarting([]() {
        printf(">>>Running trajectory auto\n");
      });
  } else {
    return FallbackAuto(driveSubsystem);
  }
}

frc2::CommandPtr autos::ProcessorSideAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem) {
  if (centerTrajectory.has_value()) {
    return SwerveTrajectoryCommand(driveSubsystem, processorSideTrajectory.value())
      .AndThen(
        coralTroughSubsystem.DispenseCoral()
      )
      .BeforeStarting([]() {
        printf(">>>Running trajectory auto\n");
      });
  } else {
    return FallbackAuto(driveSubsystem);
  }
}

frc2::CommandPtr autos::BargeSideAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem) {
  if (centerTrajectory.has_value()) {
    return SwerveTrajectoryCommand(driveSubsystem, bargeSideTrajectory.value())
      .AndThen(
        coralTroughSubsystem.DispenseCoral()
      )
      .BeforeStarting([]() {
        printf(">>>Running trajectory auto\n");
      });
  } else {
    return FallbackAuto(driveSubsystem);
  }
}


frc2::CommandPtr autos::ProcessorSideLoadAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem) {
  if (centerTrajectory.has_value()) {
    return SwerveTrajectoryCommand(driveSubsystem, processorSideTrajectory.value())
      .AndThen(
        coralTroughSubsystem.DispenseCoral()
      ).AndThen(
        SwerveTrajectoryCommand(driveSubsystem, processorSideLoadTrajectory.value()).ToPtr()
      )
      .BeforeStarting([]() {
        printf(">>>Running trajectory auto\n");
      });
  } else {
    return FallbackAuto(driveSubsystem);
  }
}

frc2::CommandPtr autos::BargeSideLoadAuto(DriveSubsystem& driveSubsystem, CoralTroughSubsystem& coralTroughSubsystem) {
  if (centerTrajectory.has_value()) {
    return SwerveTrajectoryCommand(driveSubsystem, bargeSideTrajectory.value())
      .AndThen(
        coralTroughSubsystem.DispenseCoral()
      ).AndThen(
        SwerveTrajectoryCommand(driveSubsystem, bargeSideLoadTrajectory.value()).ToPtr()
      )
      .BeforeStarting([]() {
        printf(">>>Running trajectory auto\n");
      });
  } else {
    return FallbackAuto(driveSubsystem);
  }
}