#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

class AlgaeArmSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeArmSubsystem();

  void Rotate(double power);

  frc2::CommandPtr Grab();
  frc2::CommandPtr Release();
 private:
  
  SparkMax armMotor;
  SparkMax leftRoller;
  SparkMax rightRoller;
};
