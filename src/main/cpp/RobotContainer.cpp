// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "ButtonUtil.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/NetworkButton.h>
#include <cameraserver/CameraServer.h>

RobotContainer::RobotContainer() : m_visionSubsystem(
  [this]() -> frc::Pose3d {return m_driveSubsystem.GetPose();},
  [this](frc::Pose3d measurement, units::millisecond_t timestamp) -> void {m_driveSubsystem.UpdateVisionPose(measurement, timestamp);}
) {
  // Initialize all of your commands and subsystems here

  // Stream usb camera over the network
  frc::CameraServer::StartAutomaticCapture();

  // Set up autonomous chooser
  ListAutonomousCommands();

  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand([this]() -> void {
    const auto controls = GetDriveTeleopControls();

    m_driveSubsystem.Drive(
        std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<2>(controls) * DriveConstants::kMaxTurnRate,
        std::get<3>(controls));
  }, {&m_driveSubsystem}).WithName("TeleopDrive"));

  // Configure the button bindings
  ConfigureBindings();

  nt_fastDriveSpeed = frc::Shuffleboard::GetTab("Drive")
    .Add("Max Speed", 1.0)
    .WithWidget(frc::BuiltInWidgets::kNumberSlider)
    .WithProperties({
      {"min", nt::Value::MakeDouble(0.0)},
      {"max", nt::Value::MakeDouble(1.0)}
    })
    .GetEntry();

  frc::ShuffleboardTab& mechTab = frc::Shuffleboard::GetTab("Mechanisms");

  algaeArmRaiseSpeedEntry = mechTab
    .Add("Algae Arm Raise Speed", 0.8)
    .WithWidget(frc::BuiltInWidgets::kNumberSlider)
    .WithProperties({
      {"min", nt::Value::MakeDouble(0.0)},
      {"max", nt::Value::MakeDouble(1.0)}
    })
    .GetEntry();
  
  frc2::RobotModeTriggers::Disabled().OnFalse(frc2::InstantCommand([this]() -> void {
    std::optional<frc::Pose3d> pose = m_visionSubsystem.SeedPose();
    if (pose.has_value()) {
      m_driveSubsystem.ResetPose(pose.value());
    }

    std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value()) {
      m_isRedAlliance = alliance.value() == frc::DriverStation::Alliance::kRed;
    }
  }).WithName("InitializeVision"));

  frc::Shuffleboard::GetTab("LiveWindow")
    .AddBoolean("Live Window Enabled", frc::LiveWindow::IsEnabled);

  // mechTab.Add("Algae Arm Subsystem", m_algaeArmSubsystem);
  frc::SmartDashboard::PutData("Algae Arm Subsystem", &m_algaeArmSubsystem);
  frc::SmartDashboard::PutData("Elevator Subsystem", &m_elevatorSubsystem);

  mechTab.Add("Lower Stage Quasistatic SysID", false).WithWidget(frc::BuiltInWidgets::kToggleButton);
  mechTab.Add("Lower Stage Dynamic SysID", false).WithWidget(frc::BuiltInWidgets::kToggleButton);
  mechTab.Add("Upper Stage Quasistatic SysID", false).WithWidget(frc::BuiltInWidgets::kToggleButton);
  mechTab.Add("Upper Stage Dynamic SysID", false).WithWidget(frc::BuiltInWidgets::kToggleButton);
  mechTab.Add("Coral Trough Quasistatic SysID", false).WithWidget(frc::BuiltInWidgets::kToggleButton);
  mechTab.Add("Coral Trough Dynamic SysID", false).WithWidget(frc::BuiltInWidgets::kToggleButton);

  frc2::NetworkButton(
    nt::NetworkTableInstance::GetDefault()
      .GetBooleanTopic("/Shuffleboard/Mechanisms/Lower Stage Quasistatic SysID")
  )
    .WhileTrue(
      m_elevatorSubsystem.SysIdQuasistaticLower(frc2::sysid::Direction::kForward)
        .AndThen(m_elevatorSubsystem.SysIdQuasistaticLower(frc2::sysid::Direction::kReverse))
    );
  frc2::NetworkButton(
    nt::NetworkTableInstance::GetDefault()
      .GetBooleanTopic("/Shuffleboard/Mechanisms/Lower Stage Dynamic SysID")
  )
    .WhileTrue(
      m_elevatorSubsystem.SysIdDynamicLower(frc2::sysid::Direction::kForward)
        .AndThen(m_elevatorSubsystem.SysIdDynamicLower(frc2::sysid::Direction::kReverse))
    );
  frc2::NetworkButton(
    nt::NetworkTableInstance::GetDefault()
      .GetBooleanTopic("/Shuffleboard/Mechanisms/Upper Stage Quasistatic SysID")
  )
    .WhileTrue(
      m_elevatorSubsystem.SysIdQuasistaticUpper(frc2::sysid::Direction::kForward)
        .AndThen(m_elevatorSubsystem.SysIdQuasistaticUpper(frc2::sysid::Direction::kReverse))
    );
  frc2::NetworkButton(
    nt::NetworkTableInstance::GetDefault()
      .GetBooleanTopic("/Shuffleboard/Mechanisms/Upper Stage Dynamic SysID")
  )
    .WhileTrue(
      m_elevatorSubsystem.SysIdDynamicUpper(frc2::sysid::Direction::kForward)
        .AndThen(m_elevatorSubsystem.SysIdDynamicUpper(frc2::sysid::Direction::kReverse))
    );

  frc2::NetworkButton(
    nt::NetworkTableInstance::GetDefault()
      .GetBooleanTopic("/Shuffleboard/Mechanisms/Coral Trough Quasistatic SysID")
  )
    .WhileTrue(
      m_coralTroughSubsystem.SysIdQuasistatic(frc2::sysid::Direction::kForward)
        .AndThen(m_coralTroughSubsystem.SysIdQuasistatic(frc2::sysid::Direction::kReverse))
    );
  frc2::NetworkButton(
    nt::NetworkTableInstance::GetDefault()
      .GetBooleanTopic("/Shuffleboard/Mechanisms/Coral Trough Dynamic SysID")
  )
    .WhileTrue(
      m_coralTroughSubsystem.SysIdDynamic(frc2::sysid::Direction::kForward)
        .AndThen(m_coralTroughSubsystem.SysIdDynamic(frc2::sysid::Direction::kReverse))
    );
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

  static auto steerOnlyCommand = frc2::RunCommand([this]() -> void {
    const auto controls = GetDriveTeleopControls();

    m_driveSubsystem.SteerTo(
        std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<2>(controls) * DriveConstants::kMaxTurnRate,
        std::get<3>(controls));
  }, {&m_driveSubsystem});

  frc::Shuffleboard::GetTab("Drive").Add("Steer Only", steerOnlyCommand);

  m_driverController.Back().OnTrue(frc2::InstantCommand([this]() -> void {
    m_driveSubsystem.ResetFieldOrientation(m_isRedAlliance);
  }, {&m_driveSubsystem}).IgnoringDisable(true).WithName("Reset Field Orientation"));

  m_driverController.Start().ToggleOnTrue(
    frc2::StartEndCommand(
      []() {frc::LiveWindow::SetEnabled(true);},
      []() {frc::LiveWindow::SetEnabled(false);}
    ).WithName("LiveWindow")
  );

  m_driverController.RightBumper().ToggleOnTrue(m_algaeArmSubsystem.Release());

  // m_operatorController.A().OnTrue(m_coralTroughSubsystem.LoadCoral());
  m_operatorController.B().OnTrue(m_coralTroughSubsystem.DispenseCoral());
  m_operatorController.X().ToggleOnTrue(m_algaeArmSubsystem.Grab());
  m_operatorController.Y().ToggleOnTrue(m_algaeArmSubsystem.Release());

  // TODO; use speed instead of power
  m_operatorController.RightBumper().WhileTrue(frc2::FunctionalCommand(
    []() -> void {},
    [this]() -> void {
      m_algaeArmSubsystem.Rotate(-algaeArmRaiseSpeedEntry->GetDouble(0.8) * AlgaeArmConstants::kArmSpeed);
    },
    [this](bool wasCanceled) -> void {
      m_algaeArmSubsystem.Stop();
    },
    []() -> bool {
      return false;
    },
    {&m_algaeArmSubsystem}
  ).WithName("Raise Arm"));
  m_operatorController.RightTrigger(0.05).WhileTrue(frc2::FunctionalCommand(
    []() -> void {},
    [this]() -> void {
      m_algaeArmSubsystem.Rotate(m_operatorController.GetRightTriggerAxis() * AlgaeArmConstants::kArmSpeed);
    },
    [this](bool wasCanceled) -> void {
      m_algaeArmSubsystem.Stop();
    },
    []() -> bool {
      return false;
    },
    {&m_algaeArmSubsystem}
  ).WithName("Lower Arm"));

  m_operatorController.LeftBumper().WhileTrue(m_climbSubsystem.Climb());
  m_operatorController.LeftTrigger(0.05).WhileTrue(m_climbSubsystem.Climb([this]() {
    return m_operatorController.GetLeftTriggerAxis() * ClimbConstants::kMaxClimbPower;
  }));

  m_operatorController.Back().ToggleOnTrue(m_elevatorSubsystem.ManualMove(
     [this]() -> units::meters_per_second_t {
       return ConditionRawJoystickInput(-m_operatorController.GetLeftY()) * ElevatorConstants::kOperatorSpeed;
     },
     [this]() -> units::meters_per_second_t {
       return ConditionRawJoystickInput(-m_operatorController.GetRightY()) * ElevatorConstants::kOperatorSpeed;
     }
  ));

  m_operatorController.POVDown().OnTrue(m_elevatorSubsystem.MoveToPrevious(ElevatorPointType::Algae));
  m_operatorController.POVUp().OnTrue(m_elevatorSubsystem.MoveToNext(ElevatorPointType::Algae));
  m_operatorController.POVRight().OnTrue(m_elevatorSubsystem.MoveTo(ElevatorPoint::Barge));
  m_operatorController.POVLeft().OnTrue(m_elevatorSubsystem.Home());
}

void RobotContainer::ListAutonomousCommands() {
  using namespace autos;
  m_autoChooser.SetDefaultOption("Center", AutoProgram::kCenter);
  m_autoChooser.AddOption("Team Barge", AutoProgram::kTeamBarge);
  m_autoChooser.AddOption("Opponent Barge", AutoProgram::kOpponentBarge);
  m_autoChooser.AddOption("Team Barge and Load", AutoProgram::kTeamBargeAndLoad);
  m_autoChooser.AddOption("Opponent Barge and Load", AutoProgram::kOpponentBargeAndLoad);
  frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  using namespace autos;
  switch (m_autoChooser.GetSelected()) {
    case AutoProgram::kCenter:
      return autos::CenterAuto(m_driveSubsystem, m_coralTroughSubsystem);
    case AutoProgram::kTeamBarge:
      return autos::BargeSideAuto(m_driveSubsystem, m_coralTroughSubsystem);
    case AutoProgram::kOpponentBarge:
      return autos::ProcessorSideAuto(m_driveSubsystem, m_coralTroughSubsystem);
    case AutoProgram::kTeamBargeAndLoad:
      return autos::BargeSideLoadAuto(m_driveSubsystem, m_coralTroughSubsystem);
    case AutoProgram::kOpponentBargeAndLoad:
      return autos::ProcessorSideAuto(m_driveSubsystem, m_coralTroughSubsystem);
    default:
      // Drive 1m forwards during auto
      return autos::FallbackAuto(m_driveSubsystem);
  };
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
  double leftTrigAnalogVal = m_driverController.GetLeftTriggerAxis();
  double leftStickX = -m_driverController.GetLeftY();
  double leftStickY = -m_driverController.GetLeftX();
  double rightStickRot = -m_driverController.GetRightX();

  double elevatorStability = m_elevatorSubsystem.GetStability();

  if (elevatorStability < 1.0) {
    double driveStabilityFactor = DriveConstants::kUnstableDrivePercent +
                                  (DriveConstants::kSlowDrivePercent - DriveConstants::kUnstableDrivePercent) * elevatorStability;
    leftStickX *= driveStabilityFactor;
    leftStickY *= driveStabilityFactor;
    double turnStabilityFactor = DriveConstants::kUnstableTurnPercent +
                                 (1.0 - DriveConstants::kUnstableDrivePercent) * elevatorStability;
    rightStickRot *= turnStabilityFactor;
  } else if (leftTrigAnalogVal < .05) {
    leftStickX *= DriveConstants::kSlowDrivePercent;
    leftStickY *= DriveConstants::kSlowDrivePercent;
  } else {
    double fastDrivePercent = nt_fastDriveSpeed->GetDouble(1.0);
    leftStickX *= fastDrivePercent;
    leftStickY *= fastDrivePercent;
  }

  if (m_isRedAlliance) {
    leftStickX *= -1.0;
    leftStickY *= -1.0;
  }
  

  if (m_triggerSpeedEnabled) // scale speed by analog trigger
  {
    double rightTrigAnalogVal = m_driverController.GetRightTriggerAxis();
    rightTrigAnalogVal = ConditionRawTriggerInput(rightTrigAnalogVal);

    if (leftStickX != 0 || leftStickY != 0)
    {
      if (leftStickX != 0)
      {
        double leftStickTheta = atan(leftStickY / leftStickX);
        leftStickX = rightTrigAnalogVal * cos(leftStickTheta);
        leftStickY = rightTrigAnalogVal * sin(leftStickTheta);
      }
      else
      {
        leftStickY = std::copysign(rightTrigAnalogVal, leftStickY);
      }
    }
  }
  else // scale speed by analog stick
  {
    leftStickX = ConditionRawJoystickInput(leftStickX);
    leftStickY = ConditionRawJoystickInput(leftStickY);
  }

  rightStickRot = ConditionRawJoystickInput(rightStickRot);

  return std::make_tuple(leftStickX, leftStickY, rightStickRot, m_fieldOriented);
}
