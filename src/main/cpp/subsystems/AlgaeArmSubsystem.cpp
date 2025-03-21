#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"
#include "subsystems/AlgaeArmSubsystem.h"

using namespace ctre::phoenix6;
using namespace AlgaeArmConstants;

AlgaeArmSubsystem::AlgaeArmSubsystem() :
  armMotor{kArmMotorCanID, SparkMax::MotorType::kBrushless},
  leftRoller{kLeftRollerMotorCanID},
  rightRoller{kRightRollerMotorCanID},
  armFeedforward{ArmPID::kS, ArmPID::kG, ArmPID::kV}
{
  {
    SparkMaxConfig armConfig;
    armConfig
      .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
      .SmartCurrentLimit(10.0)
      .Inverted(kArmMotorInverted);

    armConfig.encoder
      .PositionConversionFactor(kArmGearRatio)
      .VelocityConversionFactor(kArmGearRatio * 1_s / 1_min);

    armConfig.absoluteEncoder
      .Inverted(kArmEncoderInverted)
      .ZeroOffset(0.5);

    armConfig.softLimit
      .ForwardSoftLimitEnabled(true)
      .ForwardSoftLimit(kArmDownLimit.value())
      .ReverseSoftLimitEnabled(true)
      .ReverseSoftLimit(kArmUpLimit.value());

    armConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
      .Pid(ArmPID::kP, ArmPID::kI, ArmPID::kD)
      .PositionWrappingEnabled(true)
      .PositionWrappingInputRange(0.0, 1.0);

    armMotor.Configure(armConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }

  {
    using namespace ctre::phoenix6::configs;
    TalonFXSConfiguration leftRollerConfig;
    leftRollerConfig.Commutation.WithMotorArrangement(signals::MotorArrangementValue::Minion_JST);
    leftRollerConfig.CurrentLimits
      .WithStatorCurrentLimit(80.0_A)
      .WithSupplyCurrentLimit(50.0_A)
      .WithSupplyCurrentLowerLimit(40.0_A);
    leftRollerConfig.MotorOutput
      .WithInverted(signals::InvertedValue::Clockwise_Positive)
      .WithNeutralMode(signals::NeutralModeValue::Brake);

    leftRoller.GetConfigurator().Apply(leftRollerConfig);

    leftRoller.SetControl(controls::StrictFollower{rightRoller.GetDeviceID()});
    
  }

  {
    using namespace ctre::phoenix6::configs;
    TalonFXSConfiguration rightRollerConfig;
    rightRollerConfig.Commutation.WithMotorArrangement(signals::MotorArrangementValue::Minion_JST);
    rightRollerConfig.CurrentLimits
      .WithStatorCurrentLimit(80.0_A)
      .WithSupplyCurrentLimit(50.0_A)
      .WithSupplyCurrentLowerLimit(40.0_A);
    rightRollerConfig.MotorOutput
      .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
      .WithNeutralMode(signals::NeutralModeValue::Brake);

    rightRollerConfig.ExternalFeedback.WithSensorToMechanismRatio(2.0);

    rightRollerConfig.Slot0
      .WithKP(RollerPID::kP)
      .WithKI(RollerPID::kI)
      .WithKD(RollerPID::kD)
      .WithKS(RollerPID::kS)
      .WithKV(RollerPID::kV);
    
    rightRoller.GetConfigurator().Apply(rightRollerConfig);
  }
}

void AlgaeArmSubsystem::Rotate(double power) {
  armMotor.Set(power);
}

void AlgaeArmSubsystem::Stop() {
  armMotor.StopMotor();
}

frc2::CommandPtr AlgaeArmSubsystem::Grab() {
  return frc2::FunctionalCommand(
    [this]() -> void {
    },
    [this]() -> void {
      rightRoller.SetControl(controls::VelocityDutyCycle{kRollerGrabSpeed});
    },
    [this](bool wasCanceled) -> void {
      rightRoller.StopMotor();
    },
    [this]() -> bool {
      return !rightRoller.GetReverseLimit().GetValue().value;
    },
    {this}
  ).WithTimeout(kRollerGrabTimeout).WithName("Grab");
}

frc2::CommandPtr AlgaeArmSubsystem::Release() {
  return frc2::FunctionalCommand(
    [this]() -> void {
      rightRoller.SetPosition(units::turn_t{0.0});
    },
    [this]() -> void {
      rightRoller.SetControl(controls::VelocityDutyCycle{kRollerReleaseSpeed});
    },
    [this](bool wasCanceled) -> void {
      rightRoller.StopMotor();
    },
    [this]() -> bool {
      return rightRoller.GetPosition().GetValue() >= kRollerReleaseDistance;
    },
    {this}
  ).WithName("Release");
}
