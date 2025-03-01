#pragma once

#include <frc/Servo.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

class ClimbSubsystem : public frc2::SubsystemBase {
 public:
  ClimbSubsystem();

  void EngageRatchet();
  void DisengageRatchet();
  
  void Move(double power); // TODO: use speed instead
  frc2::CommandPtr Climb();

 private:
  SparkMax climbMotor;
  frc::Servo ratchetServo;
};
