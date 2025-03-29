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
      .ForwardSoftLimit(1440.0) // mm
      .ReverseSoftLimit(0.0) // mm
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimitEnabled(true);

    lowerStageConfig.closedLoop.Pid(LowerStagePID::kP, LowerStagePID::kI, LowerStagePID::kD);

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
    tab.AddString("Current ElevatorPoint", [this]() {return kPointToPointName.at(currentElevatorPoint);});
  }
}

frc2::CommandPtr ElevatorSubsystem::HomeLowerStage() {
  static double defaultCurrentLimit;
  return frc2::InstantCommand(
    [this]() {
      defaultCurrentLimit = lowerStage.configAccessor.GetSmartCurrentLimit();
      SparkFlexConfig reducedCurrentLimit;
      reducedCurrentLimit.SmartCurrentLimit(20);
      reducedCurrentLimit.softLimit.ReverseSoftLimitEnabled(false);
      lowerStage.Configure(reducedCurrentLimit, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
    },
    {this}
  ).AndThen(
    frc2::RunCommand(
      [this]() {
        lowerStage.Set(-0.05);
      },
      {this}
    ).WithTimeout(kMinHomeTime)
  ).AndThen(
    frc2::FunctionalCommand(
      []() {},
      [this]() {
        lowerStage.Set(-0.05);
      },
      [this](bool wasCanceled) {
        SparkFlexConfig regularCurrentLimit;
        regularCurrentLimit.SmartCurrentLimit(defaultCurrentLimit);
        regularCurrentLimit.softLimit.ReverseSoftLimitEnabled(true);
        lowerStage.Configure(regularCurrentLimit, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kNoPersistParameters);
      },
      [this]() -> bool {
        return lowerStage.GetEncoder().GetVelocity() > (-0.001_mps).value();
      },
      {this}
    ).ToPtr()
  );
}

frc2::CommandPtr ElevatorSubsystem::HomeUpperStage() {
  using namespace ctre::phoenix6::configs;
  using namespace ctre::phoenix6::signals;
  static CurrentLimitsConfigs currentLimitConfig;
  static SoftwareLimitSwitchConfigs softLimitConfig;
  return frc2::InstantCommand(
    [this]() {
      upperStage.GetConfigurator().Refresh(currentLimitConfig);
      upperStage.GetConfigurator().Refresh(softLimitConfig);
      CurrentLimitsConfigs reducedCurrentLimit = currentLimitConfig;
      reducedCurrentLimit.WithStatorCurrentLimit(20_A);
      upperStage.GetConfigurator().Apply(reducedCurrentLimit);
      SoftwareLimitSwitchConfigs openBottomSoftLimit = softLimitConfig;
      openBottomSoftLimit.WithReverseSoftLimitEnable(false);
      upperStage.GetConfigurator().Apply(reducedCurrentLimit);
      upperStage.GetConfigurator().Apply(openBottomSoftLimit);
    },
    {}
  ).AndThen(
    frc2::RunCommand(
      [this]() {
        upperStage.Set(-0.05);
      },
      {}
    ).WithTimeout(kMinHomeTime)
  ).AndThen(
    frc2::FunctionalCommand(
      []() {},
      [this]() {
        lowerStage.Set(-0.05);
      },
      [this](bool wasCanceled) {
        upperStage.GetConfigurator().Apply(currentLimitConfig);
        upperStage.GetConfigurator().Apply(softLimitConfig);
      },
      [this]() -> bool {
        return upperStage.GetVelocity().GetValue() > -0.001_mps / kUpperStageDistancePerRotation;
      },
      {}
    ).ToPtr()
  );
  // FIXME: Split subsystems so that requirements can be correct.
}

frc2::CommandPtr ElevatorSubsystem::Home() {
  return HomeLowerStage()
          .AlongWith(HomeUpperStage())
          .BeforeStarting([this]() -> void {currentElevatorPoint = ElevatorPoint::Home;})
          .WithName("Home");
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

void ElevatorSubsystem::MoveLowerStage(units::length::meter_t position) {
  lowerStage.GetClosedLoopController().SetReference(position.value(), SparkFlex::ControlType::kPosition);
}

void ElevatorSubsystem::MoveUpperStage(units::length::meter_t position) {
  upperStage.SetControl(controls::PositionDutyCycle(position / kUpperStageDistancePerRotation));
}

frc2::CommandPtr ElevatorSubsystem::MoveTo(ElevatorPoint point) {
  const ElevatorCoordinate pointCoordinate = kElevatorPointToElevatorCoordinate.at(point);

  return frc2::FunctionalCommand(
    [this, point]() -> void {
      currentElevatorPoint = point;
    },
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
      MoveLowerStage(pointCoordinate->lowerStagePosition);
      MoveUpperStage(pointCoordinate->upperStagePosition);
    },
    [this](bool wasCancelled) -> void {
      lowerStage.StopMotor();
      upperStage.StopMotor();
    },
    [this, pointCoordinate]() -> bool {
      return (frc::IsNear(pointCoordinate->lowerStagePosition.value(), lowerStage.GetEncoder().GetPosition(), kLowerStageMovementTolerance.value()) &&
              frc::IsNear(pointCoordinate->upperStagePosition.value(), upperStage.GetPosition().GetValue().value(), kUpperStageMovementTolerance.value()));
    },
    {this}
  ).WithName("Move To (provider)");
}

frc2::CommandPtr ElevatorSubsystem::MoveToNext(ElevatorPointType pointType) {
  return MoveTo([this, pointType]() {return GetNext(pointType);}).WithName("Move To Next");
}

frc2::CommandPtr ElevatorSubsystem::MoveToPrevious(ElevatorPointType pointType) {
  return MoveTo([this, pointType]() {return GetPrevious(pointType);}).WithName("Move To Previous");
}
