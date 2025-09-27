#include "commands/SwerveTrajectoryCommand.h"

#include <frc/DriverStation.h>

using namespace choreo;

SwerveTrajectoryCommand::SwerveTrajectoryCommand(DriveSubsystem& subsystem, Trajectory<SwerveSample>& trajectory) :
  m_driveSubsytem{subsystem}, m_trajectory{trajectory}, m_timer{}
{
  AddRequirements(&subsystem);
}

void SwerveTrajectoryCommand::Initialize() {
  m_invertForRed = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue) == frc::DriverStation::kRed;
  m_timer.Restart();
  auto initialPose = m_trajectory.GetInitialPose(m_invertForRed);
  if (initialPose.has_value()) {
    m_driveSubsytem.ResetPose(frc::Pose3d(initialPose.value()));
  }
  m_driveSubsytem.field.GetObject("traj")->SetPoses((m_invertForRed ? m_trajectory.Flipped() : m_trajectory).GetPoses());
}

void SwerveTrajectoryCommand::Execute() {
  std::optional<SwerveSample> sample = m_trajectory.SampleAt(m_timer.Get(), m_invertForRed);
  if (sample.has_value()) {
    m_driveSubsytem.FollowTrajectory(sample.value());
  }
}

void SwerveTrajectoryCommand::End(bool wasCanceled) {
  m_driveSubsytem.Stop();
  m_driveSubsytem.field.GetObject("traj")->SetPoses({});
}

bool SwerveTrajectoryCommand::IsFinished() {
  return m_timer.Get() > m_trajectory.GetTotalTime();
}
