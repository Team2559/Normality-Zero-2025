#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkBase.h>

#include <frc2/command/FunctionalCommand.h>
#include <units/angle.h>

#include "subsystems/CoralShelfSubsystem.h"
#include "Constants.h"

using namespace CoralShelfConstants;

CoralShelfSubsystem::CoralShelfSubsystem() :
  dispenserMotor{kDispenserMotorCanID, SparkMax::MotorType::kBrushless}
{
  {
    SparkMaxConfig dispenserMotorConfig;
    dispenserMotorConfig
      .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
      .SmartCurrentLimit(20.0)
      .Inverted(kDispenserMotorInverted);

    dispenserMotor.Configure(dispenserMotorConfig, SparkMax::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
  }
}

frc2::CommandPtr CoralShelfSubsystem::LoadCoral() {
  return frc2::FunctionalCommand(
    [this]() -> void {
    },
    [this]() -> void {
    },
    [this](bool wasCancelled) -> void {
    },
    [this]() -> bool {
      return false;
    },
    {this}
  ).ToPtr();
}

frc2::CommandPtr CoralShelfSubsystem::DispenseCoral() {
  return frc2::FunctionalCommand(
    [this]() -> void {
      dispenserMotor.GetEncoder().SetPosition(0.0);
    },
    [this]() -> void {
      dispenserMotor.Set(kDispenserMotorSpeed.value());
    },
    [this](bool wasCancelled) -> void {
      dispenserMotor.StopMotor();
    },
    [this]() -> bool {
      return units::turn_t{dispenserMotor.GetEncoder().GetPosition()} >= kDispenserMotorStopDistance;
    },
    {this}
  ).ToPtr();
}
