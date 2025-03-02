#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkBase.h>

#include <frc2/command/FunctionalCommand.h>
#include <units/angle.h>

#include "subsystems/CoralTroughSubsystem.h"
#include "Constants.h"

using namespace CoralTroughConstants;

CoralTroughSubsystem::CoralTroughSubsystem() :
  rollerBar{kRollerBarMotorCanID, SparkMax::MotorType::kBrushless},
  flapServo{kFlapServoPWMChannel}
{
  {
    SparkMaxConfig rollerBarConfig;
    rollerBarConfig
      .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
      .SmartCurrentLimit(20.0)
      .Inverted(kRollerBarMotorInverted);

    rollerBar.Configure(rollerBarConfig, SparkMax::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
  }
}

frc2::CommandPtr CoralTroughSubsystem::LoadCoral() {
  return frc2::FunctionalCommand(
    [this]() -> void {
    },
    [this]() -> void {
      flapServo.Set(kFlapServoDown);
    },
    [this](bool wasCancelled) -> void {
      flapServo.Set(kFlapServoUp);
    },
    [this]() -> bool {
      return false;
    },
    {this}
  ).WithTimeout(kFlapServoLowerTime);
}

frc2::CommandPtr CoralTroughSubsystem::DispenseCoral() {
  return frc2::FunctionalCommand(
    [this]() -> void {
      rollerBar.GetEncoder().SetPosition(0.0);
    },
    [this]() -> void {
      rollerBar.Set(kRollerBarDispenseSpeed.value());
    },
    [this](bool wasCancelled) -> void {
      rollerBar.StopMotor();
    },
    [this]() -> bool {
      return units::turn_t{rollerBar.GetEncoder().GetPosition()} >= kRollerBarStopDistance;
    },
    {this}
  ).ToPtr();
}
