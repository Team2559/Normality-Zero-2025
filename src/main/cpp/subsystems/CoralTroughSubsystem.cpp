#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkBase.h>

#include <frc2/command/FunctionalCommand.h>
#include <units/angle.h>
#include <frc/simulation/RoboRioSim.h>

#include "subsystems/CoralTroughSubsystem.h"
#include "Constants.h"

using namespace CoralTroughConstants;

CoralTroughSubsystem::CoralTroughSubsystem() :
  rollerBar{kRollerBarMotorCanID, SparkMax::MotorType::kBrushless},
  flapServo{kFlapServoPWMChannel},
  m_sysIdRoutine{
    frc2::sysid::Config{{}, {}, {}, nullptr},
    frc2::sysid::Mechanism{
      [this](units::volt_t driveVoltage) {
        rollerBar.SetVoltage(driveVoltage);
      },
      [this](frc::sysid::SysIdRoutineLog* log) {
        auto rollerBarVoltage = units::volt_t{rollerBar.GetBusVoltage()} * rollerBar.GetAppliedOutput();
        auto rollerBarEncoder= rollerBar.GetEncoder();
        log->Motor("coralTrough")
            .voltage(rollerBarVoltage)
            .position(units::turn_t{rollerBarEncoder.GetPosition()})
            .velocity(units::revolutions_per_minute_t{rollerBarEncoder.GetVelocity()}.convert<units::turns_per_second>());
      },
      this
    }
  },
  rollerBarSim{&rollerBar, &rollerBarMotor}
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

    rollerBar.Configure(rollerBarConfig, SparkMax::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
  }
}

frc2::CommandPtr CoralTroughSubsystem::LoadCoral() {
  return frc2::FunctionalCommand(
    []() -> void {
    },
    [this]() -> void {
      flapServo.Set(kFlapServoDown);
    },
    [this](bool wasCancelled) -> void {
      flapServo.Set(kFlapServoUp);
    },
    []() -> bool {
      return false;
    },
    {this}
  ).WithTimeout(kFlapServoLowerTime);
}

frc2::CommandPtr CoralTroughSubsystem::DispenseCoral() {
  std::shared_ptr<bool> successfullyEjected = std::make_shared<bool>(false);

  return frc2::FunctionalCommand(
    [this, successfullyEjected]() -> void {
      // Start unjamming
      rollerBar.GetEncoder().SetPosition(0.0);
      flapServo.Set(kFlapServoDejam);
      *successfullyEjected = false;
    },
    [this]() -> void {
      rollerBar.GetClosedLoopController().SetReference(-kRollerBarDispenseSpeed.value(), SparkMax::ControlType::kVelocity);
    },
    [this](bool interrupted) -> void {
      rollerBar.StopMotor();
      flapServo.Set(kFlapServoUp);
    },
    [this]() -> bool {
      return units::turn_t{rollerBar.GetEncoder().GetPosition()} <= kRollerBarPrimeDistance;
    },
    {this}
  ).AndThen(
    frc2::FunctionalCommand(
      [this]() -> void {
        // Start ejection
        rollerBar.GetEncoder().SetPosition(0.0);
      },
      [this]() -> void {
        rollerBar.GetClosedLoopController().SetReference(kRollerBarDispenseSpeed.value(), SparkMax::ControlType::kVelocity);
      },
      [this, successfullyEjected](bool interrupted) -> void {
        rollerBar.StopMotor();
        if (!interrupted) {
          *successfullyEjected = true;
        }
      },
      [this]() -> bool {
        return units::turn_t{rollerBar.GetEncoder().GetPosition()} >= kRollerBarStopDistance;
      },
      {this}
    ).WithTimeout(kDispenseTimeout)
  ).Repeatedly().Until([successfullyEjected]() -> bool {return *successfullyEjected;}).WithTimeout(kMaxDispenseTime);
}

frc2::CommandPtr CoralTroughSubsystem::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr CoralTroughSubsystem::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}

void CoralTroughSubsystem::SimulationPeriodic() {
  rollerBarPhysSim.SetInput(frc::Vectord<1>{rollerBarSim.GetAppliedOutput() * frc::sim::RoboRioSim::GetVInVoltage().value()});
  rollerBarPhysSim.Update(20_ms); // Default loop time

  rollerBarSim.iterate(
    rollerBarPhysSim.GetAngularVelocity().convert<units::revolutions_per_minute>().value(),
    frc::sim::RoboRioSim::GetVInVoltage().value(),
    (20_ms).convert<units::second>().value()
  );

  rollerBarSim.GetRelativeEncoderSim().SetPosition(rollerBarPhysSim.GetAngularPosition().convert<units::turn>().value());
  rollerBarSim.GetRelativeEncoderSim().SetVelocity(rollerBarPhysSim.GetAngularVelocity().convert<units::revolutions_per_minute>().value());
}
