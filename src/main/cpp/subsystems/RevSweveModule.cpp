#include <rev/config/SparkFlexConfig.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <wpi/sendable/Sendable.h>

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
      .VelocityConversionFactor(kDriveDistancePerRotation.value() * (1.0_s / 1.0_min));

    driveConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .Pid(DrivePID::kP, DrivePID::kI, DrivePID::kD);
    driveVff = DrivePID::kD;

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

void RevSwerveModule::TestInit(std::string name) {
  frc::ShuffleboardTab &driveSetupTab = frc::Shuffleboard::GetTab("Drive Setup");
  frc::ShuffleboardTab &steerSetupTab = frc::Shuffleboard::GetTab("Steer Setup");

  nt_driveOutput = driveSetupTab.Add(name, std::array{0.0, 0.0, 0.0}).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  nt_steerOutput = steerSetupTab.Add(name, std::array{0.0, 0.0, 0.0}).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
}

void RevSwerveModule::TestExit() {
  nt_driveOutput = {};
  nt_steerOutput = {};
}

void RevSwerveModule::TestDebug() {
  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{1.0, 2.0, 3.0});
  }
}

inline void BuildPIDConfig(SparkFlexConfig &config, const PIDUpdate &update) {
  ClosedLoopSlot slot;
  switch (update.slot) {
    case 0:
      slot = kSlot0;
      break;
    case 1:
      slot = kSlot1;
      break;
    case 2:
      slot = kSlot2;
      break;
    case 3:
      slot = kSlot3;
      break;
    default:
      return;
  }

  switch (update.term) {
  case PIDUpdate::PIDTerm::kP:
    config.closedLoop.P(update.value, slot);
    break;
  case PIDUpdate::PIDTerm::kI:
    config.closedLoop.I(update.value, slot);
    break;
  case PIDUpdate::PIDTerm::kD:
    config.closedLoop.D(update.value, slot);
    break;
  case PIDUpdate::PIDTerm::kFF:
    config.closedLoop.VelocityFF(update.value, slot);
    break;
  }
}

void RevSwerveModule::UpdateDrivePID(PIDUpdate &update) {
  if (update.term == PIDUpdate::PIDTerm::kFF) {
    driveVff = update.value;
    return;
  }
  SparkFlexConfig config;
  BuildPIDConfig(config, update);
  driveMotor.Configure(config, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
}

void RevSwerveModule::UpdateSteerPID(PIDUpdate &update) {
  SparkFlexConfig config;
  BuildPIDConfig(config, update);
  steerMotor.Configure(config, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
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
  driveMotor.GetClosedLoopController().SetReference(velocity.value(), SparkFlex::ControlType::kVelocity, {}, velocity.value() * driveVff);

  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{
      velocity.value(),
      driveEncoder.GetVelocity(),
      driveMotor.GetAppliedOutput()
    });
  }
}

void RevSwerveModule::ResetDriveEncoder() {
  driveEncoder.SetPosition(0.0);
}

void RevSwerveModule::SetDrivePercent(double percent) {
  driveMotor.Set(percent);

  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{
      0.0,
      driveEncoder.GetVelocity(),
      driveMotor.GetAppliedOutput()
    });
  }
}

void RevSwerveModule::StopDrive() {
  driveMotor.StopMotor();

  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{
      0.0,
      driveEncoder.GetVelocity(),
      driveMotor.GetAppliedOutput()
    });
  }
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
  StopDrive();
}
