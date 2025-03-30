#pragma once

#include <frc/estimator/PoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include "Constants.h"

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem(std::function<frc::Pose3d ()> accessor, std::function<void (frc::Pose3d, units::millisecond_t)> updater);

  std::optional<frc::Pose3d> SeedPose();

  void Periodic() override;

  private:
  // Camera for pose estimation
  std::unique_ptr<photon::PhotonCamera> m_camera;
  photon::PhotonPoseEstimator m_poseEstimator;

  std::function<frc::Pose3d ()> m_accessor;
  std::function<void (frc::Pose3d, units::millisecond_t)> m_updater;

  void ProcessCameraResults(std::function<void (frc::Pose3d, units::millisecond_t)> updater);
};