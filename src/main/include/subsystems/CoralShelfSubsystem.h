#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>

#include "rev/SparkMax.h"

using namespace::rev::spark;

class CoralShelfSubsystem : public frc2::SubsystemBase {
 public:
  CoralShelfSubsystem();

  frc2::CommandPtr LoadCoral();
  frc2::CommandPtr DispenseCoral();

 private:
  SparkMax dispenserMotor;
};
