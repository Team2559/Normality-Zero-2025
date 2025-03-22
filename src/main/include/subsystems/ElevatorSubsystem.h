#pragma once

#include "units/length.h"
#include <array>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix6/TalonFXS.hpp>

#include <rev/SparkFlex.h>

using namespace rev::spark;
using namespace ctre::phoenix6::hardware;

enum class ElevatorPointType {
  Algae,
  Coral,
  Any
};

enum class ElevatorPoint {
  Home,
  Processor,
  CoralL2,
  AlgaeL2,
  CoralL3,
  AlgaeL3,
  CoralL4,
  Barge
};

const std::map<ElevatorPoint, const char*> kPointToPointName = {
  {ElevatorPoint::Home, "Home"},
  {ElevatorPoint::Processor, "Processor"},
  {ElevatorPoint::CoralL2, "CoralL2"},
  {ElevatorPoint::AlgaeL2, "AlgaeL2"},
  {ElevatorPoint::CoralL3, "CoralL3"},
  {ElevatorPoint::AlgaeL3, "AlgaeL3"},
  {ElevatorPoint::CoralL4, "CoralL4"},
  {ElevatorPoint::Barge, "Barge"}
};

const std::map<ElevatorPoint, ElevatorPointType> kPointToPointType = {
  {ElevatorPoint::Home, ElevatorPointType::Any},
  {ElevatorPoint::Processor, ElevatorPointType::Algae},
  {ElevatorPoint::CoralL2, ElevatorPointType::Coral},
  {ElevatorPoint::AlgaeL2, ElevatorPointType::Algae},
  {ElevatorPoint::CoralL3, ElevatorPointType::Coral},
  {ElevatorPoint::AlgaeL3, ElevatorPointType::Algae},
  {ElevatorPoint::CoralL4, ElevatorPointType::Coral},
  {ElevatorPoint::Barge, ElevatorPointType::Algae}
};

constexpr std::array<ElevatorPoint, 9> kPointOrder = {
  ElevatorPoint::Home,
  ElevatorPoint::Processor,
  ElevatorPoint::CoralL2,
  ElevatorPoint::AlgaeL2,
  ElevatorPoint::CoralL3,
  ElevatorPoint::AlgaeL3,
  ElevatorPoint::CoralL4,
  ElevatorPoint::Barge
};

struct ElevatorCoordinate {
  units::meter_t lowerStagePosition;
  units::meter_t upperStagePosition;
};

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  frc2::CommandPtr HomeLowerStage();
  frc2::CommandPtr HomeUpperStage();
  frc2::CommandPtr Home();

  void MoveLowerStage(double power); // TODO: closed-loop speed instead of power
  void MoveUpperStage(double power);

  ElevatorPoint GetCurrent();
  ElevatorPoint GetNext(ElevatorPointType pointType);
  ElevatorPoint GetPrevious(ElevatorPointType pointType);

  frc2::CommandPtr MoveTo(ElevatorPoint point);
  frc2::CommandPtr MoveToNext(ElevatorPointType pointType);
  frc2::CommandPtr MoveToPrevious(ElevatorPointType pointType);

 private:
  SparkFlex lowerStage;
  TalonFXS upperStage;
  ElevatorPoint currentElevatorPoint;

  void MoveLowerStage(units::length::meter_t position);
  void MoveUpperStage(units::length::meter_t position);
};
