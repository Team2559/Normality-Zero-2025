#include <rev/config/SparkFlexConfig.h>

#include "subsystems/RevSwerveModule.h"
#include "Constants.h"

using namespace DriveConstants;

units::turn_t wrapOffset(units::turn_t value) {
  return value - units::math::floor(value);
}

RevSwerveModule::RevSwerveModule(int driveCanID, int steerCanID, units::angle::turn_t offset) : 
  driveMotor{driveCanID, SparkFlex::MotorType::kBrushless},
  steerMotor{steerCanID, SparkFlex::MotorType::kBrushless},
  driveEncoder{driveMotor.GetEncoder()},
  steerEncoder{steerMotor.GetAbsoluteEncoder()}
{
  {
    SparkFlexConfig driveConfig;
    driveConfig
      .SetIdleMode(SparkFlexConfig::IdleMode::kBrake)
      .SmartCurrentLimit(40.0)
      .Inverted(kDriveMotorInverted);

    driveConfig.encoder
      .PositionConversionFactor(kDriveDistancePerRotation.value())
      .VelocityConversionFactor(kDriveDistancePerRotation.value());

    driveConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .Pidf(DrivePID::kP, DrivePID::kI, DrivePID::kD, DrivePID::kFF);

    driveMotor.Configure(driveConfig, SparkFlex::ResetMode::kResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
  }
  {
    SparkFlexConfig steerConfig;
    steerConfig
      .SetIdleMode(SparkFlexConfig::IdleMode::kCoast)
      .SmartCurrentLimit(60.0)
      .Inverted(kSteerMotorInverted);

    steerConfig.absoluteEncoder
      .ZeroOffset(wrapOffset(offset).value())
      .Inverted(kSteerSensorInverted)
      .PositionConversionFactor(kSteerFeedbackScale)
      .VelocityConversionFactor(kSteerFeedbackScale);

    steerConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
      .PositionWrappingEnabled(true)
      .PositionWrappingInputRange(-0.5 * kSteerFeedbackScale, 0.5 * kSteerFeedbackScale)
      .Pid(SteerPID::kP, SteerPID::kI, SteerPID::kD);

    steerMotor.Configure(steerConfig, SparkFlex::ResetMode::kResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
  }
}

void RevSwerveModule::Periodic() {
  // Do stuff to keep module running and keep odometry in sync
}

bool RevSwerveModule::GetStatus() const {
  // TODO, actually check things
  return true;
}

void RevSwerveModule::SetSteerOffset(units::angle::turn_t offset) {
  SparkFlexConfig steerConfig;

  steerConfig.absoluteEncoder.ZeroOffset(wrapOffset(offset).value());

  steerMotor.Configure(steerConfig, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
}

units::angle::turn_t RevSwerveModule::GetSteerPosition() {
  return units::angle::turn_t{steerEncoder.GetPosition() * kInvSteerFeedbackScale};
}

void RevSwerveModule::SetSteerPosition(units::angle::turn_t position) {
  steerMotor.GetClosedLoopController().SetReference(position.value() * kSteerFeedbackScale, SparkFlex::ControlType::kPosition);
}

void RevSwerveModule::StopSteer() {
  steerMotor.Set(0.0);
}

void RevSwerveModule::SetDriveVelocity(units::velocity::meters_per_second_t velocity) {
  driveMotor.GetClosedLoopController().SetReference(velocity.value(), SparkFlex::ControlType::kVelocity);
}

void RevSwerveModule::ResetDriveEncoder() {
  driveEncoder.SetPosition(0.0);
}

void RevSwerveModule::SetDrivePercent(double percent) {
  driveMotor.Set(percent);
}

void RevSwerveModule::StopDrive() {
  driveMotor.StopMotor();
}

const frc::SwerveModuleState RevSwerveModule::GetState() {
  return {units::velocity::meters_per_second_t{driveEncoder.GetVelocity()}, GetSteerPosition()};
}

const frc::SwerveModulePosition RevSwerveModule::GetPosition() {
  return {units::length::meter_t{driveEncoder.GetPosition()}, GetSteerPosition()};
}

void RevSwerveModule::SetDesiredState(frc::SwerveModuleState &state) {
  auto currentAngle = frc::Rotation2d(GetSteerPosition());
  // Allow modules to flip their "positive" direction when rapidly changing requested directions
  state.Optimize(currentAngle);

  // Reduce speed of misoriented modules
  state.speed *= (state.angle - currentAngle).Cos();

  SetSteerPosition(state.angle.Radians());
  SetDriveVelocity(state.speed);
}

void RevSwerveModule::Stop() {
  StopSteer();
  SetDriveVelocity(0.0_mps);
}
