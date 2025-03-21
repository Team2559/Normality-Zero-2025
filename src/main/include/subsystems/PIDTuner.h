#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>
#include <rev/config/SparkBaseConfig.h>
#include <unordered_set>


struct PIDUpdate {
  enum class PIDTerm {
    kP,
    kI,
    kD,
    kFF,
  } term;

  double value;
  uint slot;
};

class PIDTuner: public wpi::Sendable, public wpi::SendableHelper<PIDTuner> {
 public:
  PIDTuner(std::function<void (PIDUpdate)> handler) :
    PIDTuner(handler, 0.0, 0.0, 0.0) {};
  PIDTuner(std::function<void (PIDUpdate)> handler, double kP, double kI, double kD) :
    PIDTuner(handler, kP, kI, kD, 0.0) {};
  PIDTuner(std::function<void (PIDUpdate)> handler, double kP, double kI, double kD, double kFF) :
    handler(handler), kP{kP}, kI{kI}, kD{kD}, kFF{kFF} {};
  ~PIDTuner() = default;

  void InitSendable(wpi::SendableBuilder &builder) override;
  uint Slot();
 private:
  std::function<void (PIDUpdate)> handler;
  uint slot = 0;

  double kP;
  double kI;
  double kD;
  double kFF;
};
