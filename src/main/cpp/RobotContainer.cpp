// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "ButtonUtil.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <cameraserver/CameraServer.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Stream usb camera over the network
  frc::CameraServer::StartAutomaticCapture();

  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand([this]() -> void {
    const auto controls = GetDriveTeleopControls();

    m_driveSubsystem.Drive(
        std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<2>(controls) * DriveConstants::kMaxTurnRate,
        std::get<3>(controls));
  }, {&m_driveSubsystem}).ToPtr());

  // Configure the button bindings
  ConfigureBindings();

  fastDriveSpeedEntry = frc::Shuffleboard::GetTab("Drive")
    .Add("Max Speed", 1.0)
    .WithWidget(frc::BuiltInWidgets::kNumberSlider)
    .WithProperties({
      {"min", nt::Value::MakeDouble(0.0)},
      {"max", nt::Value::MakeDouble(1.0)}
    })
    .GetEntry();

  algaeArmRaiseSpeedEntry = frc::Shuffleboard::GetTab("Mechanisms")
    .Add("Algae Arm Raise Speed", 0.8)
    .WithWidget(frc::BuiltInWidgets::kNumberSlider)
    .WithProperties({
      {"min", nt::Value::MakeDouble(0.0)},
      {"max", nt::Value::MakeDouble(1.0)}
    })
    .GetEntry();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_driverController.Start().ToggleOnTrue(frc2::RunCommand([this]() -> void {
    const auto controls = GetDriveTeleopControls();

    m_driveSubsystem.SteerTo(
        std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<2>(controls) * DriveConstants::kMaxTurnRate,
        std::get<3>(controls));
  }, {&m_driveSubsystem}).ToPtr());

  m_driverController.Back().OnTrue(frc2::InstantCommand([this]() -> void {
    m_driveSubsystem.ResetFieldOrientation();
  }, {&m_driveSubsystem}).ToPtr());

  m_operatorController.B().OnTrue(m_coralTroughSubsystem.DispenseCoral());
  m_operatorController.A().OnTrue(m_coralTroughSubsystem.LoadCoral());
  m_operatorController.X().ToggleOnTrue(m_algaeArmSubsystem.Grab());
  m_operatorController.Y().ToggleOnTrue(m_algaeArmSubsystem.Release());

  // TODO; use speed instead of power
  m_operatorController.RightBumper().WhileTrue(frc2::RunCommand([this]() -> void {
    m_algaeArmSubsystem.Rotate(algaeArmRaiseSpeedEntry->GetDouble(0.8));
  }, {&m_algaeArmSubsystem}).ToPtr());
  m_operatorController.RightTrigger(0.05).WhileTrue(frc2::RunCommand([this]() -> void {
    m_algaeArmSubsystem.Rotate(m_operatorController.GetRightTriggerAxis());
  }, {&m_algaeArmSubsystem}).ToPtr());

  m_operatorController.LeftBumper().WhileTrue(m_climbSubsystem.Climb());
  m_operatorController.LeftTrigger(0.05).WhileTrue(frc2::FunctionalCommand(
    [this]() -> void {
      m_climbSubsystem.DisengageRatchet();
    },
    [this]() -> void {
      m_climbSubsystem.Move(m_operatorController.GetLeftTriggerAxis() * ClimbConstants::kMaxClimbPower);
    },
    [this](bool wasCanceled) -> void {
      m_climbSubsystem.EngageRatchet();
    },
    []() -> bool {
      return false;
    },
    {&m_climbSubsystem}
  ).ToPtr());

  // m_elevatorSubsystem.SetDefaultCommand(frc2::RunCommand(
  //   [this]() -> void {
  //     m_elevatorSubsystem.MoveLowerStage(ConditionRawJoystickInput(m_operatorController.GetLeftY()));
  //     m_elevatorSubsystem.MoveUpperStage(ConditionRawJoystickInput(m_operatorController.GetLeftX()));
  //   },
  //   {&m_elevatorSubsystem}
  // ));

  // m_operatorController.POVLeft().OnTrue(m_elevatorSubsystem.Home());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Drive 1m forwards during auto
  return frc2::RunCommand([&]() -> void {
    m_driveSubsystem.Drive(-0.1_mps, 0.0_mps, 0.0_rad_per_s, true);
  }, {&m_driveSubsystem}).Until([&]() -> bool {
    return units::math::abs(m_driveSubsystem.GetPose().X()) >= 1.5_m;
  }).AndThen(
    m_coralTroughSubsystem.DispenseCoral()
  );
}

std::tuple<double, double, double, bool> RobotContainer::GetDriveTeleopControls()
{
  /*
  The robot's frame of reference is the standard unit circle, from
  trigonometry. However, the front of the robot is facing along the positive
  X axis. This means the positive Y axis extends outward from the left (or
  port) side of the robot. Positive rotation is counter-clockwise. On the
  other hand, as the controller is held, the Y axis is aligned with forward.
  And, specifically, it is the negative Y axis which extends forward. So,
  the robot's X is the controllers inverted Y. On the controller, the X
  axis lines up with the robot's Y axis. And, the controller's positive X
  extends to the right. So, the robot's Y is the controller's inverted X.
  Finally, the other controller joystick is used for commanding rotation and
  things work out so that this is also an inverted X axis.
  */
  double LeftTrigAnalogVal = m_driverController.GetLeftTriggerAxis();
  double LeftStickX = -m_driverController.GetLeftY();
  double LeftStickY = -m_driverController.GetLeftX();
  double rightStickRot = -m_driverController.GetRightX();

  if (LeftTrigAnalogVal < .05)
  {
    LeftStickX *= DriveConstants::kSlowDrivePercent;
    LeftStickY *= DriveConstants::kSlowDrivePercent;
  } else {
    double fastDrivePercent = fastDriveSpeedEntry->GetDouble(1.0);
    LeftStickX *= fastDrivePercent;
    LeftStickY *= fastDrivePercent;
  }
  

  if (m_triggerSpeedEnabled) // scale speed by analog trigger
  {
    double RightTrigAnalogVal = m_driverController.GetRightTriggerAxis();
    RightTrigAnalogVal = ConditionRawTriggerInput(RightTrigAnalogVal);

    if (LeftStickX != 0 || LeftStickY != 0)
    {
      if (LeftStickX != 0)
      {
        double LeftStickTheta = atan(LeftStickY / LeftStickX);
        LeftStickX = RightTrigAnalogVal * cos(LeftStickTheta);
        LeftStickY = RightTrigAnalogVal * sin(LeftStickTheta);
      }
      else
      {
        LeftStickY = std::copysign(RightTrigAnalogVal, LeftStickY);
      }
    }
  }
  else // scale speed by analog stick
  {
    LeftStickX = ConditionRawJoystickInput(LeftStickX);
    LeftStickY = ConditionRawJoystickInput(LeftStickY);
  }

  rightStickRot = ConditionRawJoystickInput(rightStickRot);

  return std::make_tuple(LeftStickX, LeftStickY, rightStickRot, m_fieldOriented);
}
