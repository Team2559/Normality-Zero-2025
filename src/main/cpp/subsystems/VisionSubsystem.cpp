#include "subsystems/VisionSubsystem.h"

using namespace VisionConstants;

VisionSubsystem::VisionSubsystem(std::function<frc::Pose3d ()> accessor, std::function<void (frc::Pose3d, units::millisecond_t)> updater) :
  m_camera{new photon::PhotonCamera{kName}},
  m_poseEstimator{kAprilTags, photon::MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam},
  m_accessor{accessor},
  m_updater{updater}
{
  m_poseEstimator.SetMultiTagFallbackStrategy(photon::CLOSEST_TO_REFERENCE_POSE);
}

std::optional<frc::Pose3d> VisionSubsystem::SeedPose() {
  // Switch to a fallback strategy that doesn't depend on knowing an approximate pose
  m_poseEstimator.SetMultiTagFallbackStrategy(photon::CLOSEST_TO_CAMERA_HEIGHT);
  std::optional<frc::Pose3d> seed = {};

  photon::PhotonPipelineResult camResult = m_camera->GetLatestResult();
  if (camResult.HasTargets()) {
    auto result = m_poseEstimator.Update(camResult);

    if (result.has_value()) {
      // The camera successfully captured target information
      seed = result.value().estimatedPose;
    }
  }

  // Switch back to the original fallback strategy
  m_poseEstimator.SetMultiTagFallbackStrategy(photon::CLOSEST_TO_REFERENCE_POSE);

  return seed;
}

void VisionSubsystem::Periodic() {
  m_poseEstimator.SetReferencePose(m_accessor());

  photon::PhotonPipelineResult camResult = m_camera->GetLatestResult();
  if (camResult.HasTargets()) {
    auto result = m_poseEstimator.Update(camResult);

    if (result.has_value()) {
      // The camera successfully captured target information
      m_updater(result.value().estimatedPose, result.value().timestamp);
    }
  }
}