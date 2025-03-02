#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkFlex.h>
#include <ctre/phoenix6/TalonFXS.hpp>

using namespace rev::spark;
using namespace ctre::phoenix6::hardware;

enum ElevatorPointType {
  Algae,
  Coral,
};

enum ElevatorPoint {
  Home,
  Processor,
  CoralL1,
  CoralL2,
  AlgaeL2,
  CoralL3,
  AlgaeL3,
  CoralL4,
  Barge,
};

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();
  
  frc2::CommandPtr HomeLowerStage();
  frc2::CommandPtr HomeUpperStage();
  frc2::CommandPtr Home();

  void MoveLowerStage(double power); // TODO: closed-loop speed instead of power
  void MoveUpperStage(double power);

  // frc2::CommandPtr MoveTo(ElevatorPoint point);

  // frc2::CommandPtr MoveToNext(ElevatorPointType pointType);
  // frc2::CommandPtr MoveToPrevious(ElevatorPointType pointType);

 private:
  SparkFlex lowerStage;
  TalonFXS upperStage;


};
