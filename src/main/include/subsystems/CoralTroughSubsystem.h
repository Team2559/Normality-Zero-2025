#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <rev/SparkMax.h>

#include <frc/Servo.h>

using namespace::rev::spark;

class CoralTroughSubsystem : public frc2::SubsystemBase {
 public:
  CoralTroughSubsystem();

  frc2::CommandPtr LoadCoral();
  frc2::CommandPtr DispenseCoral();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:
  SparkMax rollerBar;
  frc::Servo flapServo;

  frc2::sysid::SysIdRoutine m_sysIdRoutine;
};
