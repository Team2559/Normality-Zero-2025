#include "commands/SwerveTrajectoryCommand.h"

using namespace choreo;

SwerveTrajectoryCommand::SwerveTrajectoryCommand(DriveSubsystem& subsystem, Trajectory<SwerveSample>& trajectory) :
  m_driveSubsytem{subsystem}, m_trajectory{trajectory}, m_timer{}
{
  AddRequirements(&subsystem);
}

void SwerveTrajectoryCommand::Initialize() {
  m_timer.Restart();
  auto initialPose = m_trajectory.GetInitialPose();
  if (initialPose.has_value()) {
    m_driveSubsytem.ResetPose(frc::Pose3d(initialPose.value()));
  }
}

void SwerveTrajectoryCommand::Execute() {
  std::optional<SwerveSample> sample = m_trajectory.SampleAt(m_timer.Get());
  if (sample.has_value()) {
    m_driveSubsytem.FollowTrajectory(sample.value());
  }
}

void SwerveTrajectoryCommand::End(bool wasCanceled) {
  m_driveSubsytem.Stop();
}

bool SwerveTrajectoryCommand::IsFinished() {
  return m_timer.Get() > m_trajectory.GetTotalTime();
}
