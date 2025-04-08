#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/RevSwerveModule.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() :
  frontLeftModule{new RevSwerveModule(kFrontLeftDriveMotorCanID, kFrontLeftSteerMotorCanID, kFrontLeftSteerOffset)},
  frontRightModule{new RevSwerveModule(kFrontRightDriveMotorCanID, kFrontRightSteerMotorCanID, kFrontRightSteerOffset)},
  rearLeftModule{new RevSwerveModule(kRearLeftDriveMotorCanID, kRearLeftSteerMotorCanID, kRearLeftSteerOffset)},
  rearRightModule{new RevSwerveModule(kRearRightDriveMotorCanID, kRearRightSteerMotorCanID, kRearRightSteerOffset)},
  m_ahrs{new studica::AHRS(studica::AHRS::NavXComType::kMXP_SPI)},
  m_xController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
  m_yController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
  m_rController{OrientationPID::kP, OrientationPID::kI, OrientationPID::kD},
  m_driveTuner{[this](PIDUpdate update) -> void {
    frontLeftModule->UpdateDrivePID(update);
    frontRightModule->UpdateDrivePID(update);
    rearLeftModule->UpdateDrivePID(update);
    rearRightModule->UpdateDrivePID(update);
  }, DrivePID::kP, DrivePID::kI, DrivePID::kD, DrivePID::kV},
  m_steerTuner{[this](PIDUpdate update) -> void {
    frontLeftModule->UpdateSteerPID(update);
    frontRightModule->UpdateSteerPID(update);
    rearLeftModule->UpdateSteerPID(update);
    rearRightModule->UpdateSteerPID(update);
  }, SteerPID::kP, SteerPID::kI, SteerPID::kD}
{
  const frc::Pose3d initialPose{};

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
  nt_rPosition = rLayout.Add("Orientation [rad]", initialPose.Rotation().ToRotation2d().Radians().value()).GetEntry();
  nt_rSetpoint = rLayout.Add("Setpoint [rad]", 0.0).GetEntry();
  nt_rOutput = rLayout.Add("Output [radps]", 0.0).GetEntry();

  m_poseEstimator = std::make_unique<frc::SwerveDrivePoseEstimator3d<4>>(
    kDriveKinematics, m_ahrs->GetRotation3d(), GetModulePositions(), initialPose
  );

  // Bind test init and test exit to mode transition
  frc2::RobotModeTriggers::Test()
    .OnTrue(frc2::InstantCommand([this]() -> void {TestInit();}).AndThen(frc2::RunCommand([this]() -> void {
      frontLeftModule->TestDebug();
      frontRightModule->TestDebug();
      rearLeftModule->TestDebug();
      rearRightModule->TestDebug();
    }).WithTimeout(1.0_s)))
    .OnFalse(frc2::InstantCommand([this]() -> void {TestExit();}).ToPtr());

  frc::SmartDashboard::PutData(&field);
}

void DriveSubsystem::ResetDrive() {
  frontLeftModule->ResetDriveEncoder();
  frontRightModule->ResetDriveEncoder();
  rearLeftModule->ResetDriveEncoder();
  rearRightModule->ResetDriveEncoder();

  ResetPose(frc::Pose3d({}, GetPose().Rotation()));
}

void DriveSubsystem::Periodic() {
  frc::Rotation3d heading = m_ahrs->GetRotation3d();

  frc::Pose3d pose = m_poseEstimator->Update(heading, GetModulePositions());

  // TODO: Add Limelight update?

  nt_xPosition->SetDouble(pose.X().value());
  nt_yPosition->SetDouble(pose.Y().value());
  nt_rPosition->SetDouble(pose.Rotation().ToRotation2d().Radians().value());

  frc::SmartDashboard::PutNumber("Front left drive", frontLeftModule->GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("Front right drive", frontRightModule->GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("Rear left drive", rearLeftModule->GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("Rear right drive", rearRightModule->GetPosition().distance.value());

  frc::SmartDashboard::PutNumber("Front left steer", frontLeftModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Front right steer", frontRightModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Rear left steer", rearLeftModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Rear right steer", rearRightModule->GetSteerPosition().convert<units::deg>().value());

  field.SetRobotPose(pose.ToPose2d());
}

void DriveSubsystem::SimulationPeriodic() {

}

void DriveSubsystem::TestInit() {
  frc::ShuffleboardTab &tab = frc::Shuffleboard::GetTab("Drive");

  frc::ShuffleboardTab &driveSetupTab = frc::Shuffleboard::GetTab("Drive Setup");
  frc::ShuffleboardTab &steerSetupTab = frc::Shuffleboard::GetTab("Steer Setup");

  driveSetupTab.Add("PID", m_driveTuner).WithWidget(frc::BuiltInWidgets::kPIDController);
  steerSetupTab.Add("PID", m_steerTuner).WithWidget(frc::BuiltInWidgets::kPIDController);

  driveSetupTab.Add("Legend", "(Blue) setpoint, (Red) measured, (Green) output");
  steerSetupTab.Add("Legend", "(Blue) setpoint, (Red) measured, (Green) output");

  frontLeftModule->TestInit("Front left");
  frontRightModule->TestInit("Front right");
  rearLeftModule->TestInit("Rear left");
  rearRightModule->TestInit("Rear right");
}

void DriveSubsystem::TestExit() {
  frontLeftModule->TestExit();
  frontRightModule->TestExit();
  rearLeftModule->TestExit();
  rearRightModule->TestExit();
}

void DriveSubsystem::ResetFieldOrientation(bool inverted) {
  m_poseEstimator->ResetPose(frc::Pose3d(GetPose().Translation(), inverted ? frc::Rotation3d(frc::Rotation2d(180_deg)): frc::Rotation3d()));
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center)
{
  frc::Rotation2d heading = GetPose().ToPose2d().Rotation();

  nt_xOutput->SetDouble(xSpeed.value());
  nt_yOutput->SetDouble(ySpeed.value());
  nt_rOutput->SetDouble(rot.value());

  SetModuleStates(kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading)
                  : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
    {x_center, y_center}
  ));
}

void DriveSubsystem::FollowTrajectory(const choreo::SwerveSample& sample) {
  frc::Pose2d pose = GetPose().ToPose2d();

  units::meters_per_second_t xFeedback{m_xController.Calculate(pose.X().value(), sample.x.value())};
  units::meters_per_second_t yFeedback{m_yController.Calculate(pose.Y().value(), sample.y.value())};
  units::radians_per_second_t rotFeedback{
      m_rController.Calculate(pose.Rotation().Radians().value(), sample.heading.value())
  };

  nt_xSetpoint->SetDouble(sample.x.value());
  nt_ySetpoint->SetDouble(sample.y.value());
  nt_rSetpoint->SetDouble(sample.heading.value());

  Drive(
    sample.vx + xFeedback,
    sample.vy + yFeedback,
    sample.omega + rotFeedback,
    true
  );
}

void DriveSubsystem::SteerTo(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center)
{
  frc::Rotation2d heading = GetPose().ToPose2d().Rotation();

  SetModuleStates(kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading)
                  : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
    {x_center, y_center}
  ), true);
}

void DriveSubsystem::Stop() {
  nt_xOutput->SetDouble(0.0);
  nt_yOutput->SetDouble(0.0);
  nt_rOutput->SetDouble(0.0);

  frontLeftModule->Stop();
  frontRightModule->Stop();
  rearLeftModule->Stop();
  rearRightModule->Stop();
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

void DriveSubsystem::ResetPose(frc::Pose3d pose) {
  m_poseEstimator->ResetPose(pose);
}

frc::Pose3d DriveSubsystem::GetPose() {
  return m_poseEstimator->GetEstimatedPosition();
}

void DriveSubsystem::UpdateVisionPose(frc::Pose3d measurement, units::millisecond_t timestamp) {
  m_poseEstimator->AddVisionMeasurement(measurement, timestamp, {0.5, 0.5, 0.5, 0.8});
}