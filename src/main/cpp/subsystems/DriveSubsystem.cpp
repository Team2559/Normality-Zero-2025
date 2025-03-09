#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/RevSwerveModule.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() :
  frontLeftModule{new RevSwerveModule(kFrontLeftDriveMotorCanID, kFrontLeftSteerMotorCanID, kFrontLeftSteerOffset)},
  frontRightModule{new RevSwerveModule(kFrontRightDriveMotorCanID, kFrontRightSteerMotorCanID, kFrontRightSteerOffset)},
  rearLeftModule{new RevSwerveModule(kRearLeftDriveMotorCanID, kRearLeftSteerMotorCanID, kRearLeftSteerOffset)},
  rearRightModule{new RevSwerveModule(kRearRightDriveMotorCanID, kRearRightSteerMotorCanID, kRearRightSteerOffset)},
  m_ahrs{new studica::AHRS(studica::AHRS::NavXComType::kMXP_SPI)}
{
  const frc::Pose2d initialPose{};

  // TODO: Add Limelight init, also get initial pose from LL if available?

  frc::ShuffleboardTab &tab = frc::Shuffleboard::GetTab("Drive");

  frc::ShuffleboardLayout &xLayout = tab.GetLayout("X", frc::BuiltInLayouts::kList);
  nt_xPosition = xLayout.Add("Position [m]", initialPose.X().value()).GetEntry();
  nt_xSetpoint = xLayout.Add("Setpoint [m]", 0.0).GetEntry();
  nt_xOutput = xLayout.Add("Output [mps]", 0.0).GetEntry();

  frc::ShuffleboardLayout &yLayout = tab.GetLayout("Y", frc::BuiltInLayouts::kList);
  nt_yPosition = yLayout.Add("Position [m]", initialPose.Y().value()).GetEntry();
  nt_ySetpoint = yLayout.Add("Setpoint [m]", 0.0).GetEntry();
  nt_yOutput = yLayout.Add("Output [mps]", 0.0).GetEntry();

  frc::ShuffleboardLayout &rLayout = tab.GetLayout("R", frc::BuiltInLayouts::kList);
  nt_rPosition = rLayout.Add("Orientation [rad]", initialPose.Rotation().Radians().value()).GetEntry();
  nt_rSetpoint = rLayout.Add("Setpoint [rad]", 0.0).GetEntry();
  nt_rOutput = rLayout.Add("Output [radps]", 0.0).GetEntry();

  m_poseEstimator = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(
    kDriveKinematics, m_ahrs->GetRotation2d(), GetModulePositions(), initialPose
  );
}

void DriveSubsystem::ResetDrive() {
  frontLeftModule->ResetDriveEncoder();
  frontRightModule->ResetDriveEncoder();
  rearLeftModule->ResetDriveEncoder();
  rearRightModule->ResetDriveEncoder();
}

void DriveSubsystem::Periodic() {
  frc::Rotation2d heading = m_ahrs->GetRotation2d();

  frc::Pose2d pose = m_poseEstimator->Update(heading, GetModulePositions());

  // TODO: Add Limelight update?

  nt_xPosition->SetDouble(pose.X().value());
  nt_yPosition->SetDouble(pose.Y().value());
  nt_rPosition->SetDouble(pose.Rotation().Radians().value());

  frc::SmartDashboard::PutNumber("Front left steer", frontLeftModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Front right steer", frontRightModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Rear left steer", rearLeftModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Rear right steer", rearRightModule->GetSteerPosition().convert<units::deg>().value());
}

void DriveSubsystem::SimulationPeriodic() {

}

void DriveSubsystem::ResetFieldOrientation() {
  m_ahrs->ZeroYaw();
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center)
{
  frc::Rotation2d heading = m_ahrs->GetRotation2d();

  nt_xOutput->SetDouble(xSpeed.value());
  nt_yOutput->SetDouble(ySpeed.value());
  nt_rOutput->SetDouble(rot.value());

  SetModuleStates(kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading)
                  : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
    {x_center, y_center}
  ));
}

void DriveSubsystem::SteerTo(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center)
{
  frc::Rotation2d heading = m_ahrs->GetRotation2d();

  SetModuleStates(kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading)
                  : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
    {x_center, y_center}
  ), true);
}

const std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions()
{
  return {
      frontLeftModule->GetPosition(),
      frontRightModule->GetPosition(),
      rearLeftModule->GetPosition(),
      rearRightModule->GetPosition()
  };
}

void DriveSubsystem::SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates, bool steerOnly)
{
  auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

  if (frontLeft.speed == 0.0_mps &&
      frontRight.speed == 0.0_mps &&
      rearLeft.speed == 0.0_mps &&
      rearRight.speed == 0.0_mps)
  {
    frontLeftModule->Stop();
    frontRightModule->Stop();
    rearLeftModule->Stop();
    rearRightModule->Stop();

    return;
  }

  if (!steerOnly) {
    frontLeftModule->SetDesiredState(frontLeft);
    frontRightModule->SetDesiredState(frontRight);
    rearLeftModule->SetDesiredState(rearLeft);
    rearRightModule->SetDesiredState(rearRight);
  } else {
    frontLeftModule->SetSteerPosition(frontLeft.angle.Radians());
    frontRightModule->SetSteerPosition(frontRight.angle.Radians());
    rearLeftModule->SetSteerPosition(rearLeft.angle.Radians());
    rearRightModule->SetSteerPosition(rearRight.angle.Radians());
  }
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_poseEstimator->GetEstimatedPosition();
}

