#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
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

    rollerBarConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .Pidf(RollerPID::kP, RollerPID::kI, RollerPID::kD, RollerPID::kFF);

    frc::SmartDashboard::PutString("Coral Status", "Initialized");

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
      flapServo.Set(kFlapServoDejam);
      frc::SmartDashboard::PutString("Coral Status", "Starting prime");
    },
    [this]() -> void {
      rollerBar.GetClosedLoopController().SetReference(-kRollerBarDispenseSpeed.value(), SparkMax::ControlType::kVelocity);
    },
    [this](bool wasCancelled) -> void {
      rollerBar.StopMotor();
      flapServo.Set(kFlapServoUp);
      frc::SmartDashboard::PutString("Coral Status", "Finishing prime");
    },
    [this]() -> bool {
      return units::turn_t{rollerBar.GetEncoder().GetPosition()} <= kRollerBarPrimeDistance;
    },
    {this}
  ).AndThen(
    frc2::FunctionalCommand(
      [this]() -> void {
        rollerBar.GetEncoder().SetPosition(0.0);
        frc::SmartDashboard::PutString("Coral Status", "Starting dispense");
      },
      [this]() -> void {
        rollerBar.GetClosedLoopController().SetReference(kRollerBarDispenseSpeed.value(), SparkMax::ControlType::kVelocity);
      },
      [this](bool wasCancelled) -> void {
        rollerBar.StopMotor();
        frc::SmartDashboard::PutString("Coral Status", "Finishing dispense");
      },
      [this]() -> bool {
        return units::turn_t{rollerBar.GetEncoder().GetPosition()} >= kRollerBarStopDistance;
      },
      {this}
    ).ToPtr()
  ).WithTimeout(5.0_s).AndThen([]() {frc::SmartDashboard::PutString("Coral Status", "Finished command");});
}
