// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

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
    .WithProperties({ // specify widget properties here
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
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Drive 1m forwards during auto
  return frc2::RunCommand([&]() -> void {
    m_driveSubsystem.Drive(1.0_mps, 0.0_mps, 0.0_rad_per_s, true);
  }, {&m_driveSubsystem}).Until([&]() -> bool {
    return units::math::abs(m_driveSubsystem.GetPose().X()) >= 1.0_m;
  });
}

double ConditionRawTriggerInput(double RawTrigVal) noexcept
{
  // Input deadband around 0.0 (+/- range).
  double deadZoneVal = 0.05;
  double deadZoneCorrection = 1.0 / (1.0 - deadZoneVal);

  if (RawTrigVal < deadZoneVal)
  {
    // Trigger is within the deadzone
    return 0;
  }
  else
  {
    // Trigger is outside the deadzone, scale the trigger value to make low magnitudes more sensitive
    RawTrigVal -= deadZoneVal;
    RawTrigVal *= deadZoneCorrection;
    return std::pow(RawTrigVal, 3.0); // Cube the trigger value
  }
}

double ConditionRawJoystickInput(double RawJoystickVal, double mixer = 0.75) noexcept
{
  /*
  Add some deadzone, so the robot doesn't drive when the joysticks are released
  and return to "zero". These implement a continuous deadband, one in which
  the full range of outputs may be generated, once joysticks move outside the
  deadband.

  Also, cube the result, to provide more operator control. Just cubing the raw
  value does a pretty good job with the deadband, but doing both is easy and
  guarantees no movement in the deadband. Cubing makes it easier to command
  smaller/slower movements, while still being able to command full power. The
  'mixer` parameter specifies what percentage of contribution towards the 
  output the cubed value has, with the remainder coming from the linear term.
  */

  // Input deadband around 0.0 (+/- range).
  constexpr double deadZoneVal = 0.05;
  constexpr double deadZoneCorrection = 1.0 / (1.0 - deadZoneVal);

  if (RawJoystickVal >= -deadZoneVal && RawJoystickVal <= +deadZoneVal)
  {
    // Stick is within deadzone
    RawJoystickVal = 0.0;
  }
  else if (RawJoystickVal < -deadZoneVal)
  {
    // Stick is "below" deadzone
    RawJoystickVal += deadZoneVal;
    RawJoystickVal *= deadZoneCorrection;
  }
  else if (RawJoystickVal > +deadZoneVal)
  {
    // Stick is "above" deadzone
    RawJoystickVal -= deadZoneVal;
    RawJoystickVal *= deadZoneCorrection;
  }

  // Cube the joystick value, and do a percentage mix with the unscaled value
  return mixer * std::pow(RawJoystickVal, 3.0) + (1.0 - mixer) * RawJoystickVal;
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
