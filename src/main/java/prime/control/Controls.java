// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package prime.control;

/** Add your docs here. */
public class Controls {

  // Controllers
  public static final int DRIVER_PORT = 0;
  public static final int OPERATOR_PORT = 1;

  // XboxOne Joysticks
  public static final int LEFT_STICK_X = 0;
  public static final int LEFT_STICK_Y = 1;
  public static final int LEFT_TRIGGER = 2;
  public static final int RIGHT_TRIGGER = 3;
  public static final int RIGHT_STICK_X = 4;
  public static final int RIGHT_STICK_Y = 5;

  // XboxOne Buttons
  public static final int A = 1;
  public static final int B = 2;
  public static final int X = 3;
  public static final int Y = 4;
  public static final int LB = 5;
  public static final int RB = 6;
  public static final int LOGO_LEFT = 7;
  public static final int LOGO_RIGHT = 8;
  public static final int LEFT_STICK_BUTTON = 9;
  public static final int RIGHT_STICK_BUTTON = 10;

  // POV Buttons
  public static final int up = 0;
  public static final int down = 180;
  public static final int right = 270;
  public static final int left = 90;

  // deadbands
  public static final double AXIS_DEADBAND = 0.1;
  public static final double ShoulderSpeedControlThreshold = 0.2;

  public static double scaleLinear(double input) {
    return Math.abs(input) / input;
  }

  public static double scaleCubic(double input, double weight) {
    return ((weight * (Math.pow(input, 3))) + ((1.0 - weight) * input));
  }

  /**
   * @param input The original input value. Range: -1 to 1
   * @param cutoff The deadband cutoff
   * @return Linearly-scaled input without a cutoff jump
   */
  public static double linearScaledDeadband(double input, double cutoff) {
    var abs = Math.abs(input);
    var linearInput = scaleLinear(input);

    if (abs < cutoff) return 0;

    return ((input - (linearInput * cutoff)) / (1 - cutoff));
  }

  /**
   * @param input The original input value (-1 - 1)
   * @param cutoff The deadband cutoff (0 - 1)
   * @param weight The weight of the cubic curve (0 - 1)
   * @return Cubic-scaled input without a cutoff jump
   */
  public static double cubicScaledDeadband(
    double input,
    double cutoff,
    double weight
  ) {
    var abs = Math.abs(input);
    var cubicInput = scaleCubic(input, weight);
    var cubicCutoff = scaleCubic(cutoff, weight);

    if (abs < cutoff) return 0;

    return ((cubicInput - (abs / input) * cubicCutoff) / (1.0 - cubicCutoff));
  }
}
