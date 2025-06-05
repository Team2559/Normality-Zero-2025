#include "subsystems/PIDTuner.h"

void PIDTuner::InitSendable(wpi::SendableBuilder &builder) {
  builder.SetSmartDashboardType("PIDController");
  builder.AddDoubleProperty(
    "p", [this]() -> double {
      return kP;
    }, [this](double kP) -> void {
      this->kP = kP;
      handler({PIDUpdate::PIDTerm::kP, kP, slot});
    }
  );
  builder.AddDoubleProperty(
    "i", [this]() -> double {
      return kI;
    }, [this](double kI) -> void {
      this->kI = kI;
      handler({PIDUpdate::PIDTerm::kI, kI, slot});
    }
  );
  builder.AddDoubleProperty(
    "d", [this]() -> double {
      return kD;
    }, [this](double kD) -> void {
      this->kD = kD;
      handler({PIDUpdate::PIDTerm::kD, kD, slot});
    }
  );
  builder.AddDoubleProperty(
    "f", [this]() -> double {
      return kFF;
    }, [this](double kFF) -> void {
      this->kFF = kFF;
      handler({PIDUpdate::PIDTerm::kFF, kFF, slot});
    }
  );
  builder.AddIntegerProperty(
    "slot", [this]() -> int {
      return slot;
    }, [this](int newSlot) -> void {
      slot = newSlot;
    }
  );
}

uint PIDTuner::Slot() {
  return slot;
}
