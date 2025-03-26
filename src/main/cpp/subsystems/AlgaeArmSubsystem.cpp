#include <frc/shuffleboard/Shuffleboard.h>
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
      .SmartCurrentLimit(30.0)
      .Inverted(kArmMotorInverted);

    armConfig.encoder
      .PositionConversionFactor(kArmGearRatio)
      .VelocityConversionFactor(kArmGearRatio * 1_s / 1_min);

    armConfig.absoluteEncoder
      .Inverted(kArmEncoderInverted)
      .ZeroOffset(0.25);

    // armConfig.softLimit
    //   .ForwardSoftLimitEnabled(true)
    //   .ForwardSoftLimit(kArmDownLimit.value())
    //   .ReverseSoftLimitEnabled(true)
    //   .ReverseSoftLimit(kArmUpLimit.value());

    armConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
      .Pid(ArmPID::kP, ArmPID::kI, ArmPID::kD)
      .PositionWrappingEnabled(true)
      .PositionWrappingInputRange(-0.5, 0.5);

    armMotor.Configure(armConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }

  m_armTarget = units::turn_t{armMotor.GetAbsoluteEncoder().GetPosition()};

  {
    using namespace ctre::phoenix6::configs;
    TalonFXSConfiguration leftRollerConfig;
    leftRollerConfig.Commutation.WithMotorArrangement(signals::MotorArrangementValue::Minion_JST);
    leftRollerConfig.CurrentLimits
      .WithStatorCurrentLimit(80.0_A)
      .WithSupplyCurrentLimit(50.0_A)
      .WithSupplyCurrentLowerLimit(40.0_A);
    leftRollerConfig.MotorOutput
      .WithInverted(kLeftRollerInverted)
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
      .WithInverted(kRightRollerInverted)
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

  frc::ShuffleboardTab &mechanismsTab = frc::Shuffleboard::GetTab("Mechanisms");

  mechanismsTab.AddDouble("Algae arm target", [this]() {return m_armTarget.value();});
  mechanismsTab.AddDouble("Algae arm sensor", [this]() {return armMotor.GetAbsoluteEncoder().GetPosition();});
}

void AlgaeArmSubsystem::Periodic() {
  units::second_t currentLoop = frc::Timer::GetFPGATimestamp();
  m_loopDelta = currentLoop - m_lastLoop;
  m_lastLoop = currentLoop;

  units::volt_t feedforward = armFeedforward.Calculate(m_armTarget, m_armTargetVel);
  armMotor.GetClosedLoopController().SetReference(m_armTarget.value(), SparkMax::ControlType::kPosition, {}, feedforward.value());
}

void AlgaeArmSubsystem::Rotate(units::turns_per_second_t speed) {
  m_armTargetVel = -speed;
  m_armTarget += -speed * m_loopDelta;
  // Clamp arm target to be within the physical range
  m_armTarget = units::math::min(units::math::max(m_armTarget, kArmUpLimit), kArmDownLimit);
}

void AlgaeArmSubsystem::Stop() {
  m_armTargetVel = 0.0_deg_per_s;
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
