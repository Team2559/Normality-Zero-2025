#include <algorithm>

#include <array>
#include <cstdlib>

#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/MathUtil.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include <rev/config/ClosedLoopConfig.h>
#include <rev/config/SparkFlexConfig.h>

#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"

using namespace ElevatorConstants;
using namespace ctre::phoenix6;
using namespace rev::spark;

ElevatorSubsystem::ElevatorSubsystem() :
  lowerStage{kLowerStageMotorCanID, SparkFlex::MotorType::kBrushless},
  upperStage{kUpperStageMotorCanID}
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
      .ForwardSoftLimit(760.0) // mm
      .ReverseSoftLimit(0.0) // mm
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimitEnabled(true);

    lowerStageConfig.closedLoop.Pidf(LowerStagePID::kP, LowerStagePID::kI, LowerStagePID::kD, LowerStagePID::kFF);

    lowerStage.Configure(lowerStageConfig, SparkFlex::ResetMode::kResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
  }
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
      .WithForwardSoftLimitThreshold(20_in / kUpperStageDistancePerRotation)
      .WithReverseSoftLimitThreshold(0_deg)
      .WithForwardSoftLimitEnable(true)
      .WithReverseSoftLimitEnable(true);

    // upperStageConfig.Slot0
    //   .WithGravityType(GravityTypeValue::Elevator_Static)
    //   .WithKP(UpperStagePID::kP)
    //   .WithKI(UpperStagePID::kI)
    //   .WithKD(UpperStagePID::kD)
    //   .WithKV(UpperStagePID::kV)
    //   .WithKG(UpperStagePID::kG);

    upperStage.GetConfigurator().Apply(upperStageConfig);
  }
  {
    frc::ShuffleboardTab &tab = frc::Shuffleboard::GetTab("Mechanisms");

    tab.AddDouble("Lower-Stage Position", [this]() {return lowerStage.GetEncoder().GetPosition();});
    tab.AddDouble("Upper-Stage Position", [this]() {return upperStage.GetPosition().GetValue().value();});
  }
}

frc2::CommandPtr ElevatorSubsystem::HomeLowerStage() {
  static double defaultCurrentLimit;
  return frc2::FunctionalCommand(
    [this]() -> void {
      defaultCurrentLimit = lowerStage.configAccessor.GetSmartCurrentLimit();
      SparkFlexConfig reducedCurrentLimit;
      reducedCurrentLimit.SmartCurrentLimit(20);
      reducedCurrentLimit.softLimit.ReverseSoftLimitEnabled(false);
      lowerStage.Configure(reducedCurrentLimit, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
    },
    [this]() -> void {
      lowerStage.Set(-0.05);
    },
    [this](bool wasCanceled) -> void {
      SparkFlexConfig regularCurrentLimit;
      regularCurrentLimit.SmartCurrentLimit(defaultCurrentLimit);
      regularCurrentLimit.softLimit.ReverseSoftLimitEnabled(true);
      lowerStage.Configure(regularCurrentLimit, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
    },
    [this]() -> bool {
      return lowerStage.GetEncoder().GetVelocity() > -1;
    },
    {this}
  ).ToPtr();
}

frc2::CommandPtr ElevatorSubsystem::HomeUpperStage() {
  using namespace ctre::phoenix6::configs;
  using namespace ctre::phoenix6::signals;
  // static double defaultCurrentLimit;
  return frc2::FunctionalCommand(
    [this]() -> void {
      // defaultCurrentLimit = upperStage;
      TalonFXSConfiguration reducedCurrentLimit;
      reducedCurrentLimit.CurrentLimits.WithStatorCurrentLimit(20_A);
      reducedCurrentLimit.SoftwareLimitSwitch.WithReverseSoftLimitEnable(false);
      upperStage.GetConfigurator().Apply(reducedCurrentLimit);
    },
    [this]() -> void {
      lowerStage.Set(-0.05);
    },
    [this](bool wasCanceled) -> void {
      TalonFXSConfiguration reducedCurrentLimit;
      reducedCurrentLimit.CurrentLimits.WithStatorCurrentLimit(80_A);
      reducedCurrentLimit.SoftwareLimitSwitch.WithReverseSoftLimitEnable(true);
      upperStage.GetConfigurator().Apply(reducedCurrentLimit);
    },
    [this]() -> bool {
      return upperStage.GetVelocity().GetValue() > -0.001_mps / kUpperStageDistancePerRotation;
    },
    {}
  ).ToPtr();
  // FIXME: Split subsystems so that requirements can be correct.
}

frc2::CommandPtr ElevatorSubsystem::Home() {
  return HomeLowerStage().AlongWith(HomeUpperStage());
}

void ElevatorSubsystem::MoveLowerStage(double power) {
  lowerStage.Set(power);
}

void ElevatorSubsystem::MoveUpperStage(double power) {
  upperStage.Set(power * 0.5);
}

ElevatorPoint ElevatorSubsystem::GetCurrent() {
  return currentElevatorPoint;
}

std::function<bool(ElevatorPoint point)> IsMatchingElevatorPointType(ElevatorPointType pointType) {
  return [pointType](ElevatorPoint point) -> bool {
    const ElevatorPointType pointTypeOfPoint = kPointToPointType.at(point);

    return (pointTypeOfPoint == pointType) || (pointTypeOfPoint == ElevatorPointType::Any) || (pointType == ElevatorPointType::Any);
  };
}

ElevatorPoint ElevatorSubsystem::GetNext(ElevatorPointType pointType) {
  std::array<const ElevatorPoint, 9>::iterator currentPointTypeIterator = std::ranges::find(kPointOrder.begin(), kPointOrder.end(), currentElevatorPoint);

  if (currentPointTypeIterator == kPointOrder.end())
    return currentElevatorPoint;

  currentPointTypeIterator = std::ranges::find_if(currentPointTypeIterator++, kPointOrder.end(), IsMatchingElevatorPointType(pointType));

  if (currentPointTypeIterator == kPointOrder.end())
    return currentElevatorPoint;
  else
    return *currentPointTypeIterator;
}

ElevatorPoint ElevatorSubsystem::GetPrevious(ElevatorPointType pointType) {
  std::array<const ElevatorPoint, 9>::reverse_iterator currentPointTypeIterator = std::ranges::find(kPointOrder.rbegin(), kPointOrder.rend(), currentElevatorPoint);

  if (currentPointTypeIterator == kPointOrder.rend())
    return currentElevatorPoint;

  currentPointTypeIterator = std::ranges::find_if(currentPointTypeIterator++, kPointOrder.rend(), IsMatchingElevatorPointType(pointType));

  if (currentPointTypeIterator == kPointOrder.rend())
    return currentElevatorPoint;
  else
    return *currentPointTypeIterator;
}

void ElevatorSubsystem::MoveLowerStage(units::length::meter_t position) {
  lowerStage.GetClosedLoopController().SetReference(position.value(), SparkFlex::ControlType::kPosition);
}

void ElevatorSubsystem::MoveUpperStage(units::length::meter_t position) {
  upperStage.SetControl(controls::PositionDutyCycle(position / kUpperStageDistancePerRotation));
}

frc2::CommandPtr ElevatorSubsystem::MoveTo(ElevatorPoint point) {
  const ElevatorCoordinate pointCoordinate = kElevatorPointToElevatorCoordinate.at(point);

  return frc2::FunctionalCommand(
    []() -> void {},
    [this, pointCoordinate]() -> void {
      MoveLowerStage(pointCoordinate.lowerStagePosition);
      MoveUpperStage(pointCoordinate.upperStagePosition);
    },
    [this](bool wasCancelled) -> void {
      lowerStage.StopMotor();
      upperStage.StopMotor();
    },
    [this, pointCoordinate]() -> bool {
      return (frc::IsNear(pointCoordinate.lowerStagePosition.value(), lowerStage.GetEncoder().GetPosition(), kLowerStageMovementTolerance.value()) &&
              frc::IsNear(pointCoordinate.upperStagePosition.value(), upperStage.GetPosition().GetValue().value(), kUpperStageMovementTolerance.value()));
    },
    {this}
  ).ToPtr();
}

frc2::CommandPtr ElevatorSubsystem::MoveToNext(ElevatorPointType pointType) {
  return MoveTo(GetNext(pointType));
}

frc2::CommandPtr ElevatorSubsystem::MoveToPrevious(ElevatorPointType pointType) {
  return MoveTo(GetPrevious(pointType));
}
