#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <ctre/phoenix6/TalonFXS.hpp>

using namespace rev::spark;
using namespace ctre::phoenix6::hardware;

class AlgaeArmSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeArmSubsystem();

  void Rotate(double power);
  void Stop();

  frc2::CommandPtr Grab();
  frc2::CommandPtr Release();
 private:

  SparkMax armMotor;
  TalonFXS leftRoller;
  TalonFXS rightRoller;

  frc::ArmFeedforward armFeedforward;
};
