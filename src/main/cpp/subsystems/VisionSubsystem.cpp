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

  ProcessCameraResults([&seed](frc::Pose3d pose, units::millisecond_t timestamp) {
    seed = pose;
  });

  // Switch back to the original fallback strategy
  m_poseEstimator.SetMultiTagFallbackStrategy(photon::CLOSEST_TO_REFERENCE_POSE);

  return seed;
}

void VisionSubsystem::Periodic() {
  m_poseEstimator.SetReferencePose(m_accessor());

  ProcessCameraResults(m_updater);
}

void VisionSubsystem::ProcessCameraResults(std::function<void (frc::Pose3d, units::millisecond_t)> updater) {
  std::vector<photon::PhotonPipelineResult> camResults = m_camera->GetAllUnreadResults();
  for (auto camResult : camResults) {
    if (camResult.HasTargets()) {
      auto result = m_poseEstimator.Update(camResult);

      if (result.has_value()) {
        // The camera successfully captured target information
        updater(result.value().estimatedPose, result.value().timestamp);
      }
    }
  }
}