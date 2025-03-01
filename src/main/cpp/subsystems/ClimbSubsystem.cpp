#include <frc2/command/FunctionalCommand.h>
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

    climbMotor.Configure(climbMotorConfig, SparkMax::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
  }

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