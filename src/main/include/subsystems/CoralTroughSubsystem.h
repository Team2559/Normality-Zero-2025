#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>

#include <frc/Servo.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>

#include <units/moment_of_inertia.h>

using namespace::rev::spark;

class CoralTroughSubsystem : public frc2::SubsystemBase {
 public:
  CoralTroughSubsystem();

  frc2::CommandPtr LoadCoral();
  frc2::CommandPtr DispenseCoral();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  void SimulationPeriodic() override;

 private:
  SparkMax rollerBar;
  frc::Servo flapServo;

  frc2::sysid::SysIdRoutine m_sysIdRoutine;

  frc::DCMotor rollerBarMotor = frc::DCMotor::NEO550();
  SparkMaxSim rollerBarSim;
  frc::LinearSystem<2, 1, 2> rollerBarPlant = frc::LinearSystemId::DCMotorSystem(
    rollerBarMotor,
    0.0005_kg_sq_m, // Moment of inertia of roller bar (estimated)
    1.0            // No gearing
  );
  frc::sim::DCMotorSim rollerBarPhysSim = frc::sim::DCMotorSim(rollerBarPlant, rollerBarMotor, {0.001, 0.001});
};
