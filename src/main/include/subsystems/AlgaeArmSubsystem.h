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

  void Periodic() override;

  void Rotate(units::turns_per_second_t speed);
  void Stop();

  frc2::CommandPtr Grab();
  frc2::CommandPtr Release();
 private:

  SparkMax armMotor;
  SparkRelativeEncoder armEncoder;
  SparkAbsoluteEncoder armAbsEncoder;
  TalonFXS leftRoller;
  TalonFXS rightRoller;

  frc::ArmFeedforward armFeedforward;
  units::turn_t m_armTarget;
  units::turns_per_second_t m_armTargetVel;

  units::second_t m_lastLoop;
  units::second_t m_loopDelta;
};
