#include <math.h>

#include "ButtonUtil.h"

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
    return pow(RawTrigVal, 3.0); // Cube the trigger value
  }
}

double ConditionRawJoystickInput(double RawJoystickVal, double mixer) noexcept
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
  return mixer * pow(RawJoystickVal, 3.0) + (1.0 - mixer) * RawJoystickVal;
}