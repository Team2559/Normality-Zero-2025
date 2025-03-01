#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"
#include "subsystems/AlgaeArmSubsystem.h"

using namespace AlgaeArmConstants;

AlgaeArmSubsystem::AlgaeArmSubsystem() :
  armMotor{kArmMotorCanID, SparkMax::MotorType::kBrushless},
  leftRoller{kLeftRollerMotorCanID, SparkMax::MotorType::kBrushless},
  rightRoller{kRightRollerMotorCanID, SparkMax::MotorType::kBrushless} {
    {
      SparkMaxConfig armConfig;
      armConfig
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .SmartCurrentLimit(20.0)
        .Inverted(kArmMotorInverted);

      // TODO: Setup encoder and soft limits

      armMotor.Configure(armConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    }

    {
      SparkMaxConfig leftRollerConfig;
      leftRollerConfig
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .SmartCurrentLimit(20.0)
        .Inverted(kLeftRollerInverted)
        .Follow(rightRoller);

      leftRoller.Configure(leftRollerConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    }

    {
      SparkMaxConfig rightRollerConfig;
      rightRollerConfig
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .SmartCurrentLimit(20.0)
        .Inverted(kLeftRollerInverted);

      rightRollerConfig.limitSwitch
        .ReverseLimitSwitchType(LimitSwitchConfig::kNormallyOpen)
        .ReverseLimitSwitchEnabled(true);

      rightRoller.Configure(rightRollerConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    }
}

void AlgaeArmSubsystem::Rotate(double power) {
  armMotor.Set(power);
}

frc2::CommandPtr AlgaeArmSubsystem::Grab() {
  return frc2::FunctionalCommand(
    [this]() -> void {
    },
    [this]() -> void {
      rightRoller.Set(kRollerGrabSpeed.value());
    },
    [this](bool wasCanceled) -> void {
      rightRoller.StopMotor();
    },
    [this]() -> bool {
      return rightRoller.GetReverseLimitSwitch().Get();
    },
    {this}
  ).WithTimeout(kRollerGrabTimeout);
}

frc2::CommandPtr AlgaeArmSubsystem::Release() {
  return frc2::FunctionalCommand(
    [this]() -> void {
      rightRoller.GetEncoder().SetPosition(0.0);
    },
    [this]() -> void {
      rightRoller.Set(kRollerReleaseSpeed.value());
    },
    [this](bool wasCanceled) -> void {
      rightRoller.StopMotor();
    },
    [this]() -> bool {
      return units::turn_t{rightRoller.GetEncoder().GetPosition()} >= kRollerReleaseDistance;
    },
    {this}
  ).ToPtr();
}
