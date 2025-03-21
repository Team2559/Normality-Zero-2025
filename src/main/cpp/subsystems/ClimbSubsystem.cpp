#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <rev/config/SparkMaxConfig.h>

#include "subsystems/ClimbSubsystem.h"
#include "Constants.h"

using namespace ClimbConstants;

ClimbSubsystem::ClimbSubsystem() :
  climbMotor{kClimbMotorCanID, SparkMax::MotorType::kBrushless},
  ratchetServo{kRatchetServoPWMChannel}
{
  {
    SparkMaxConfig climbMotorConfig;
    climbMotorConfig
      .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
      .SmartCurrentLimit(60.0)
      .Inverted(kClimbMotorInverted);

    climbMotor.Configure(climbMotorConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }

  frc2::Trigger disabled = frc2::RobotModeTriggers::Disabled();
  disabled.OnFalse(frc2::InstantCommand([this]() {
    SparkMaxConfig climbMotorConfig;
    climbMotorConfig.SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
    climbMotor.Configure(climbMotorConfig, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }, {this}).ToPtr());
  disabled.OnTrue(frc2::InstantCommand([this]() {
    SparkMaxConfig climbMotorConfig;
    climbMotorConfig.SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
    climbMotor.Configure(climbMotorConfig, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }, {this}).ToPtr());

  ratchetServo.SetOffline();
}

void ClimbSubsystem::EngageRatchet() {
  ratchetServo.Set(kRatchetEngaged);
  climbMotor.StopMotor();
}

void ClimbSubsystem::DisengageRatchet() {
  ratchetServo.Set(kRatchetDisengaged);
}

void ClimbSubsystem::Move(double power) {
  climbMotor.Set(power);
}

frc2::CommandPtr ClimbSubsystem::Climb() {
  return frc2::FunctionalCommand(
    [this]() -> void {
      ratchetServo.Set(kRatchetDisengaged);
    },
    [this]() -> void {
      climbMotor.Set(kClimbPower);
    },
    [this](bool wasCanceled) -> void {
      ratchetServo.Set(kRatchetEngaged);
      climbMotor.StopMotor();
    },
    [this]() -> bool {
      return false;
    },
    {this}
  ).ToPtr();
}

frc2::CommandPtr ClimbSubsystem::Climb(std::function<double()> powerProvider) {
  return frc2::FunctionalCommand(
    [this]() -> void {
      ratchetServo.Set(kRatchetDisengaged);
    },
    [this, powerProvider]() -> void {
      climbMotor.Set(powerProvider());
    },
    [this](bool wasCanceled) -> void {
      ratchetServo.Set(kRatchetEngaged);
      climbMotor.StopMotor();
    },
    [this]() -> bool {
      return false;
    },
    {this}
  ).ToPtr();
}