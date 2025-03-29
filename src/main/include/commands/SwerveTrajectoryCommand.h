#pragma once

#include <choreo/Choreo.h>
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class SwerveTrajectoryCommand : public frc2::CommandHelper<frc2::Command, SwerveTrajectoryCommand> {
 public:
  explicit SwerveTrajectoryCommand(DriveSubsystem& subsystem, choreo::Trajectory<choreo::SwerveSample>& trajectory);

  void Initialize() override;

  void Execute() override;

  void End(bool wasCanceled) override;

  bool IsFinished() override;

 private:
  DriveSubsystem& m_driveSubsytem;
  choreo::Trajectory<choreo::SwerveSample>& m_trajectory;

  frc::Timer m_timer;
};
