// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/time.h"
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

  inline constexpr int kDriverControllerPort = 0;
  inline constexpr int kOperatorControllerPort = 1;

}

namespace DriveConstants {
  // Drivebase geometry: distance between centers of right and left wheels on
  // robot; distance between centers of front and back wheels on robot.
  inline constexpr units::meter_t kWheelbaseWidth = 0.7_m;
  inline constexpr units::meter_t kWheelbaseLength = 0.7_m;

  // Zero positions for the steer of the swerve modules
  inline constexpr units::degree_t kFrontLeftSteerOffset  = 17.2_deg;
  inline constexpr units::degree_t kFrontRightSteerOffset = 89.2_deg;
  inline constexpr units::degree_t kRearLeftSteerOffset   = 325.6_deg;
  inline constexpr units::degree_t kRearRightSteerOffset  = 244.4_deg;

  // SDS Mk3 Standard (or Fast) Gear Ratio: 8.16:1 (or 6.86:1);
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 8.16 / 0.3192 => ~25.57.

  // SDS Mk4 L1, L2, L3, L4 Gear Ratio: 8.14:1, 6.75:1, 6.12:1, 5.14:1
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 6.75 / 0.3192 => ~21.15.

  // SDS Mk4n L1, L2, L3 Gear Ratio: 7.13:1, 5.90:1, 5.36:1
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 5.36 / 0.3192 => ~16.79.

  // This should be empirically determined!  This is just an initial guess.
  // This is used for both distance and velocity control. If this is off, it
  // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
  inline constexpr units::meter_t kDriveDistancePerRotation = 1.0_m / 18.48;

  // SDS Mk3 Standard (or Fast) Max Free Speed: 12.1 (or 14.4) feet/second;
  // Review your motor and swerve module configuration for nominal free speed
  // This is an upper bound, for various reasons. It needs to be empirically
  // measured. Half of theoretical free speed is a reasonable starting value
  // (since something in the ballpark is needed here in order to to drive).
  constexpr units::meters_per_second_t kMaxDriveSpeed = 22.1_fps / 2.0;
  constexpr double kSlowDrivePercent = 0.50;

  // This is used for rotating the robot in place, about it's center.  This
  // may need to be empirically adjusted, but check kDriveMetersPerRotation
  // before making any adjustment here.
  const units::meter_t kDriveMetersPerSteerCircle = 2.0_m * M_PI * pow(pow(kWheelbaseLength.value(), 2.0) + pow(kWheelbaseWidth.value(), 2.0), 0.5);

  // This is the maximum rotational speed -- not of a swerve module, but of
  // the entire robot.  This is a function of the maximum drive speed and the
  // geometry of the robot.  This will occur when the robot spins in place,
  // around the center of a circle which passes through all the drive modules
  // (if there is no single such circle, things are analogous).  If the drive
  // modules are turned to be tangential to this circle and run at maximum,
  // the robot is rotating as fast as possible.  This can be derived from
  // kMaxDriveSpeed and the geometry and does not have to be directly
  // measured.  It is a good idea to check this value empirically though.

  // So the maximum rotational velocity (spinning in place) is kMaxDriveSpeed
  // / kDriveMetersPerSteerCircle * 360 degrees.  This should not need to
  // be empirically adjusted (but check).
  const units::degrees_per_second_t kMaxTurnRate =
      kMaxDriveSpeed / kDriveMetersPerSteerCircle * 360.0_deg;

  // CAN ID assignments.
  constexpr int kFrontLeftDriveMotorCanID = 1;
  constexpr int kFrontLeftSteerMotorCanID = 2;
  constexpr int kRearLeftDriveMotorCanID = 3;
  constexpr int kRearLeftSteerMotorCanID = 4;
  constexpr int kRearRightDriveMotorCanID = 5;
  constexpr int kRearRightSteerMotorCanID = 6;
  constexpr int kFrontRightDriveMotorCanID = 7;
  constexpr int kFrontRightSteerMotorCanID = 8;
  
  // These can flip because of gearing.
  constexpr bool kDriveMotorInverted = false;
  constexpr bool kSteerMotorInverted = true;
  constexpr bool kSteerSensorInverted = false;

  // Closed loop feedback parameters for module drive speed
  namespace DrivePID {
    constexpr double kP = 0.000;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
    constexpr double kFF = (1.0 / ((565.0_rpm).convert<units::turns_per_second>() / 1.0_V) / kDriveDistancePerRotation).value();
  }

  // Steer encoder units are scaled for more responsive PID feedback
  constexpr double kSteerFeedbackScale = 10.0;
  constexpr double kInvSteerFeedbackScale = 1 / kSteerFeedbackScale;

  // Closed loop feedback parameters for module steer position
  namespace SteerPID {
    constexpr double kP = 0.8;
    constexpr double kI = 0.0;
    constexpr double kD = 0.03;
    constexpr double kFF = 0.0;
  }

  // TODO: Closed loop feedback for chassis speed and orientation
}

namespace ClimbConstants {
  constexpr int kClimbMotorCanID = 9;
  constexpr int kRatchetServoPWMChannel = 0;

  constexpr bool kClimbMotorInverted = true;
  constexpr double kClimbPower = -0.4;
  constexpr double kMaxClimbPower = 0.5;

  constexpr double kRatchetEngaged = 0.4;
  constexpr double kRatchetDisengaged = 0.2;
}

namespace ElevatorConstants {
  constexpr int kLowerStageMotorCanID = 10;
  constexpr int kUpperStageMotorCanID = 11;
}

namespace CoralTroughConstants {
  constexpr int kRollerBarMotorCanID = 12;
  constexpr int kFlapServoPWMChannel = 9;

  constexpr bool kRollerBarMotorInverted = true;

  constexpr double kFlapServoDown = 0.0;
  constexpr double kFlapServoUp = 0.642857142857;

  constexpr units::second_t kFlapServoLowerTime = 3_s;

  constexpr units::turns_per_second_t kRollerBarDispenseSpeed { 0.8 };
  constexpr units::turn_t kRollerBarStopDistance { 6.0 };
}

namespace CoralShelfConstants {
  constexpr int kDispenserMotorCanID = 13;

  constexpr bool kDispenserMotorInverted = false;

  constexpr units::turns_per_second_t kDispenserMotorSpeed { 60.0 };

  constexpr units::turn_t kDispenserMotorStopDistance { 6.0 };
}

namespace AlgaeArmConstants {
  constexpr int kArmMotorCanID = 14;
  constexpr int kLeftRollerMotorCanID = 15;
  constexpr int kRightRollerMotorCanID = 16;

  constexpr bool kArmMotorInverted = false;
  constexpr bool kLeftRollerInverted = false;
  constexpr bool kRightRollerInverted = true;

  constexpr units::turn_t kArmUpPos = 0.0_deg;
  constexpr units::turn_t kArmDownPos = 90.0_deg;
  constexpr units::turns_per_second_t kArmUpSpeed = 60.0_deg_per_s; // TODO: not yet used

  constexpr units::turns_per_second_t kRollerGrabSpeed { -3.0 }; // TODO: Interpreted as a power
  constexpr units::second_t kRollerGrabTimeout = 5_s;
  constexpr units::turns_per_second_t kRollerReleaseSpeed { 3.0 }; // TODO: Interpreted as a power
  constexpr units::turn_t kRollerReleaseDistance { 8.0 };

  namespace ArmPID {
    constexpr double kP = 0.000;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
    constexpr double kFF = (1.0 / ((917.0_rpm).convert<units::turns_per_second>() / 1.0_V)).value();
  }
}
