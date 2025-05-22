#include <algorithm>

#include <array>
#include <cstdio>
#include <cstdlib>

#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/MathUtil.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <rev/config/ClosedLoopConfig.h>
#include <rev/config/SparkFlexConfig.h>

#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"

using namespace ElevatorConstants;
using namespace ctre::phoenix6;
using namespace rev::spark;

ElevatorSubsystem::ElevatorSubsystem() {
  frc::ShuffleboardTab &tab = frc::Shuffleboard::GetTab("Mechanisms");

  tab.AddDouble("Lower-Stage Position", [this]() {return lowerStage.GetPosition().value();});
  tab.AddDouble("Upper-Stage Position", [this]() {return upperStage.GetPosition().value();});
  tab.AddString("Current ElevatorPoint", [this]() {return kPointToPointName.at(currentElevatorPoint);});
}

ElevatorSubsystem::LowerElevatorSubsystem::LowerElevatorSubsystem() :
  stageMotor{kLowerStageMotorCanID, SparkFlex::MotorType::kBrushless},
  stageEncoder{stageMotor.GetEncoder()},
  m_stageFeedforward{LowerStagePID::kS, LowerStagePID::kG, LowerStagePID::kV},
  nt_lowerStageTargetPosition{frc::Shuffleboard::GetTab("Mechanisms").Add("Lower-Stage Target Position", 0.0).GetEntry()}
{
  {
    SparkFlexConfig lowerStageConfig;
    lowerStageConfig
      .SetIdleMode(SparkFlexConfig::IdleMode::kBrake)
      .SmartCurrentLimit(80.0)
      .Inverted(kLowerStageInverted);

    lowerStageConfig.encoder
      .PositionConversionFactor(kLowerStageDistancePerRotation.value())
      .VelocityConversionFactor(kLowerStageDistancePerRotation.value() / 60.0);

    lowerStageConfig.softLimit
      .ForwardSoftLimit(kLowerStageMaxHeight.value()) // m
      .ReverseSoftLimit(0.0) // m
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimitEnabled(true);

    lowerStageConfig.closedLoop.Pid(LowerStagePID::kP, LowerStagePID::kI, LowerStagePID::kD);

    stageMotor.Configure(lowerStageConfig, SparkFlex::ResetMode::kResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
  }
}

ElevatorSubsystem::UpperElevatorSubsystem::UpperElevatorSubsystem():
  stageMotor{kUpperStageMotorCanID},
  stagePosition{stageMotor.GetPosition()},
  nt_upperStageTargetPosition{frc::Shuffleboard::GetTab("Mechanisms").Add("Upper-Stage Target Position", 0.0).GetEntry()}
{
  {
    using namespace ctre::phoenix6::configs;
    using namespace ctre::phoenix6::signals;
    TalonFXSConfiguration upperStageConfig;
    upperStageConfig.Commutation.WithMotorArrangement(MotorArrangementValue::Minion_JST);
    upperStageConfig.CurrentLimits
      .WithStatorCurrentLimit(80.0_A)
      .WithSupplyCurrentLimit(50.0_A)
      .WithSupplyCurrentLowerLimit(40.0_A);
    upperStageConfig.MotorOutput
      .WithInverted(InvertedValue::Clockwise_Positive)
      .WithNeutralMode(NeutralModeValue::Brake);

    upperStageConfig.SoftwareLimitSwitch
      .WithForwardSoftLimitThreshold(kUpperStageMaxHeight / kUpperStageDistancePerRotation)
      .WithReverseSoftLimitThreshold(0_deg)
      .WithForwardSoftLimitEnable(true)
      .WithReverseSoftLimitEnable(true);

    upperStageConfig.MotionMagic
      .WithMotionMagicCruiseVelocity(kMaxSpeed / kUpperStageDistancePerRotation);
      // .WithMotionMagicAcceleration(kMaxAccel / kUpperStageDistancePerRotation)
      // .WithMotionMagicJerk(kMaxJerk / kUpperStageDistancePerRotation);

    upperStageConfig.Slot0
      .WithGravityType(GravityTypeValue::Elevator_Static)
      .WithKP(UpperStagePID::kP)
      .WithKI(UpperStagePID::kI)
      .WithKD(UpperStagePID::kD)
      .WithKG(UpperStagePID::kG)
      .WithKS(UpperStagePID::kS)
      .WithKV(UpperStagePID::kV);

    stageMotor.GetConfigurator().Apply(upperStageConfig);
  }
}

frc2::CommandPtr ElevatorSubsystem::HomeLowerStage() {
  return lowerStage.Home();
}

frc2::CommandPtr ElevatorSubsystem::LowerElevatorSubsystem::Home() {
  static double defaultCurrentLimit;
  return frc2::InstantCommand(
    [this]() {
      defaultCurrentLimit = stageMotor.configAccessor.GetSmartCurrentLimit();
      SparkFlexConfig reducedCurrentLimit;
      reducedCurrentLimit.SmartCurrentLimit(20);
      reducedCurrentLimit.softLimit.ReverseSoftLimitEnabled(false);
      stageMotor.Configure(reducedCurrentLimit, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
    },
    {this}
  ).AndThen(
    frc2::RunCommand(
      [this]() {
        stageMotor.Set(-0.05);
      },
      {this}
    ).WithTimeout(kMinHomeTime)
  ).AndThen(
    frc2::FunctionalCommand(
      []() {},
      [this]() {
        stageMotor.Set(-0.05);
      },
      [this](bool wasCanceled) {
        stageEncoder.SetPosition(0.0);
        stageMotor.StopMotor();
        SparkFlexConfig regularCurrentLimit;
        regularCurrentLimit.SmartCurrentLimit(defaultCurrentLimit);
        regularCurrentLimit.softLimit.ReverseSoftLimitEnabled(true);
        stageMotor.Configure(regularCurrentLimit, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
      },
      [this]() -> bool {
        return stageEncoder.GetVelocity() > (-0.002_mps).value();
      },
      {this}
    ).ToPtr()
  );
}

frc2::CommandPtr ElevatorSubsystem::HomeUpperStage() {
  return upperStage.Home();
}

frc2::CommandPtr ElevatorSubsystem::UpperElevatorSubsystem::Home() {
  using namespace ctre::phoenix6::configs;
  using namespace ctre::phoenix6::signals;
  static CurrentLimitsConfigs currentLimitConfig;
  static SoftwareLimitSwitchConfigs softLimitConfig;
  return frc2::InstantCommand(
    [this]() {
      stageMotor.GetConfigurator().Refresh(currentLimitConfig);
      stageMotor.GetConfigurator().Refresh(softLimitConfig);
      CurrentLimitsConfigs reducedCurrentLimit = currentLimitConfig;
      reducedCurrentLimit.WithStatorCurrentLimit(20_A);
      SoftwareLimitSwitchConfigs openBottomSoftLimit = softLimitConfig;
      openBottomSoftLimit.WithReverseSoftLimitEnable(false);
      stageMotor.GetConfigurator().Apply(reducedCurrentLimit);
      stageMotor.GetConfigurator().Apply(openBottomSoftLimit);
    },
    {this}
  ).AndThen(
    frc2::RunCommand(
      [this]() {
        stageMotor.SetControl(controls::DutyCycleOut(-0.03));
      },
      {this}
    ).WithTimeout(kMinHomeTime)
  ).AndThen(
    frc2::FunctionalCommand(
      []() {},
      [this]() {
        stageMotor.SetControl(controls::DutyCycleOut(-0.03));
      },
      [this](bool wasCanceled) {
        stageMotor.SetPosition(0.0_rad);
        stageMotor.StopMotor();
        stageMotor.GetConfigurator().Apply(currentLimitConfig);
        stageMotor.GetConfigurator().Apply(softLimitConfig);
      },
      [this]() -> bool {
        return stageMotor.GetVelocity().GetValue() > -0.001_mps / kUpperStageDistancePerRotation;
      },
      {this}
    ).ToPtr()
  );
}

frc2::CommandPtr ElevatorSubsystem::Home() {
  return HomeLowerStage()
          .AlongWith(HomeUpperStage())
          .BeforeStarting([this]() -> void {currentElevatorPoint = ElevatorPoint::Home;})
          .WithName("Home");
}

void ElevatorSubsystem::MoveLowerStage(double power) {
  lowerStage.Move(power);
}

void ElevatorSubsystem::LowerElevatorSubsystem::Move(double power) {
  stageMotor.Set(power);
}

void ElevatorSubsystem::MoveUpperStage(double power) {
  upperStage.Move(power * 0.5);
}

void ElevatorSubsystem::UpperElevatorSubsystem::Move(double power) {
  stageMotor.Set(power);
}

frc2::CommandPtr ElevatorSubsystem::ManualMove(std::function<units::meters_per_second_t ()> lowerProvider, std::function<units::meters_per_second_t ()> upperProvider) {
  static units::meter_t lowerTarget;
  static units::meter_t upperTarget;
  return frc2::FunctionalCommand(
    [this]() {
      lowerTarget = lowerStage.GetPosition();
      upperTarget = upperStage.GetPosition();
    },
    [this, lowerProvider, upperProvider]() {
      lowerTarget += lowerProvider() * 20_ms;
      upperTarget += upperProvider() * 20_ms;
      lowerTarget = units::math::max(units::math::min(lowerTarget, kLowerStageMaxHeight), 0.0_m);
      upperTarget = units::math::max(units::math::min(upperTarget, kUpperStageMaxHeight), 0.0_m);
      // lowerStage.MoveTo(lowerTarget);
      upperStage.MoveTo(upperTarget);
    },
    [this](bool wasCanceled) {
      lowerStage.Stop();
      upperStage.Stop();
    },
    []() -> bool {
      return false;
    },
    {this, &lowerStage, &upperStage}
  ).WithName("Manual Elevator");
}

ElevatorPoint ElevatorSubsystem::GetCurrent() {
  return currentElevatorPoint;
}

std::function<bool(ElevatorPoint point)> IsMatchingElevatorPointType(ElevatorPointType pointType) {
  return [pointType](ElevatorPoint point) -> bool {
    const ElevatorPointType pointTypeOfPoint = kPointToPointType.at(point);

    return (pointTypeOfPoint == pointType) ||
           (pointTypeOfPoint == ElevatorPointType::Any) ||
           (pointType == ElevatorPointType::Any);
  };
}

ElevatorPoint ElevatorSubsystem::GetNext(ElevatorPointType pointType) {
  std::array<const ElevatorPoint, 9>::iterator elevatorPointIterator = std::ranges::find(kPointOrder.begin(), kPointOrder.end(), currentElevatorPoint);

  if (elevatorPointIterator == kPointOrder.end())
    return currentElevatorPoint;

  elevatorPointIterator = std::ranges::find_if(++elevatorPointIterator, kPointOrder.end(), IsMatchingElevatorPointType(pointType));

  if (elevatorPointIterator == kPointOrder.end())
    return currentElevatorPoint;
  else
    return *elevatorPointIterator;
}

ElevatorPoint ElevatorSubsystem::GetPrevious(ElevatorPointType pointType) {
  std::array<const ElevatorPoint, 9>::reverse_iterator elevatorPointIterator = std::ranges::find(kPointOrder.rbegin(), kPointOrder.rend(), currentElevatorPoint);

  if (elevatorPointIterator == kPointOrder.rend())
    return currentElevatorPoint;

  elevatorPointIterator = std::ranges::find_if(++elevatorPointIterator, kPointOrder.rend(), IsMatchingElevatorPointType(pointType));

  if (elevatorPointIterator == kPointOrder.rend())
    return currentElevatorPoint;
  else
    return *elevatorPointIterator;
}

void ElevatorSubsystem::LowerElevatorSubsystem::MoveTo(units::length::meter_t position) {
  units::meters_per_second_t m_lowerStageVelocity = (position - m_target) / 20_ms;
  m_target = position;
  units::volt_t feedforward = m_stageFeedforward.Calculate(m_lowerStageVelocity);
  stageMotor.GetClosedLoopController().SetReference(position.value(), SparkFlex::ControlType::kPosition, {}, feedforward.value());
  nt_lowerStageTargetPosition->SetDouble(position.value());
}

void ElevatorSubsystem::UpperElevatorSubsystem::MoveTo(units::length::meter_t position) {
  stageMotor.SetControl(controls::MotionMagicVoltage(position / kUpperStageDistancePerRotation));
  nt_upperStageTargetPosition->SetDouble(position.value());
}

void ElevatorSubsystem::LowerElevatorSubsystem::Stop() {
  stageMotor.StopMotor();
}

void ElevatorSubsystem::UpperElevatorSubsystem::Stop() {
  stageMotor.StopMotor();
}

frc2::CommandPtr ElevatorSubsystem::MoveTo(ElevatorPoint point) {
  const ElevatorCoordinate pointCoordinate = kElevatorPointToElevatorCoordinate.at(point);

  return frc2::FunctionalCommand(
    [this, point]() -> void {
      currentElevatorPoint = point;
    },
    [this, pointCoordinate]() -> void {
      lowerStage.MoveTo(pointCoordinate.lowerStagePosition);
      upperStage.MoveTo(pointCoordinate.upperStagePosition);
    },
    [this](bool wasCancelled) -> void {
      lowerStage.Stop();
      upperStage.Stop();
    },
    [this, pointCoordinate]() -> bool {
      return (frc::IsNear(pointCoordinate.lowerStagePosition, lowerStage.GetPosition(), kLowerStageMovementTolerance) &&
              frc::IsNear(pointCoordinate.upperStagePosition, upperStage.GetPosition(), kUpperStageMovementTolerance));
    },
    {this, &lowerStage, &upperStage}
  ).WithName("Move To");
}

frc2::CommandPtr ElevatorSubsystem::MoveTo(std::function<ElevatorPoint()> pointProvider) {
  std::shared_ptr<ElevatorCoordinate> pointCoordinate = std::make_shared<ElevatorCoordinate>();

  return frc2::FunctionalCommand(
    [this, pointProvider, pointCoordinate]() -> void {
      currentElevatorPoint = pointProvider();
      *pointCoordinate = kElevatorPointToElevatorCoordinate.at(currentElevatorPoint);
    },
    [this, pointCoordinate]() -> void {
      lowerStage.MoveTo(pointCoordinate->lowerStagePosition);
      upperStage.MoveTo(pointCoordinate->upperStagePosition);
    },
    [this](bool wasCancelled) -> void {
      lowerStage.Stop();
      upperStage.Stop();
    },
    [this, pointCoordinate]() -> bool {
      return (frc::IsNear(pointCoordinate->lowerStagePosition, lowerStage.GetPosition(), kLowerStageMovementTolerance) &&
              frc::IsNear(pointCoordinate->upperStagePosition, upperStage.GetPosition(), kUpperStageMovementTolerance));
    },
    {this, &lowerStage, &upperStage}
  ).WithName("Move To (provider)");
}

frc2::CommandPtr ElevatorSubsystem::MoveToNext(ElevatorPointType pointType) {
  return MoveTo([this, pointType]() {return GetNext(pointType);}).WithName("Move To Next");
}

frc2::CommandPtr ElevatorSubsystem::MoveToPrevious(ElevatorPointType pointType) {
  return MoveTo([this, pointType]() {return GetPrevious(pointType);}).WithName("Move To Previous");
}

units::meter_t ElevatorSubsystem::LowerElevatorSubsystem::GetPosition() {
  return units::meter_t{stageEncoder.GetPosition()};
}

units::meter_t ElevatorSubsystem::UpperElevatorSubsystem::GetPosition() {
  stagePosition.Refresh();
  return stagePosition.GetValue() * kUpperStageDistancePerRotation;
}
