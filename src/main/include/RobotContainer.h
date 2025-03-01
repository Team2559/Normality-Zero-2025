// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <networktables/GenericEntry.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/CoralTroughSubsystem.h"
#include "subsystems/AlgaeArmSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandXboxController m_driverController {
    OperatorConstants::kDriverControllerPort
  };

  frc2::CommandXboxController m_operatorController {
    OperatorConstants::kOperatorControllerPort
  };

  nt::GenericEntry* fastDriveSpeedEntry;
  nt::GenericEntry* algaeArmRaiseSpeedEntry;

  bool m_fieldOriented = true;
  bool m_triggerSpeedEnabled = false;

  // The robot's subsystems are defined here...
  DriveSubsystem m_driveSubsystem;
  ClimbSubsystem m_climbSubsystem;
  AlgaeArmSubsystem m_algaeArmSubsystem;
  CoralTroughSubsystem m_coralTroughSubsystem;

  void ConfigureBindings();

  std::tuple<double, double, double, bool> GetDriveTeleopControls();
};
