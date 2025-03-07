package org.prime.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.SwerveMap;

public class SwerveControlSuppliers {

  // The number of samples for which the input filters operate over
  private static final int _filterSamples = 5;

  private MedianFilter _medianFilterX = new MedianFilter(_filterSamples);
  private MedianFilter _medianFilterY = new MedianFilter(_filterSamples);
  
  private LinearFilter _linearFilterX = LinearFilter.movingAverage(_filterSamples);
  private LinearFilter _linearFilterY = LinearFilter.movingAverage(_filterSamples);

  public DoubleSupplier X;
  public DoubleSupplier Y;
  public DoubleSupplier Z;

  public SwerveControlSuppliers(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
    X = xSupplier;
    Y = ySupplier;
    Z = zSupplier;
  }

  /**
   * Gets the robot or field-relative ChassisSpeeds from the control suppliers. See 
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system 
   * for more information about robot and field-oriented control.
   * @param controlSuppliers The user input suppliers
   * @param robotRelative Whether the speeds should be robot-relative
   * @param gyroAngle The current gyro angle
   * @param disableAutoAlign A runnable to disable auto-align
   */
  public ChassisSpeeds getChassisSpeeds(boolean robotRelative, Rotation2d gyroAngle, Runnable disableAutoAlign) {
    // If the driver is trying to rotate the robot, disable snap-to control
    if (Math.abs(Z.getAsDouble()) > 0.2) {
      disableAutoAlign.run();
    }

    // Apply a median filter to X and Y
    var medianFilteredX = _medianFilterX.calculate(X.getAsDouble());
    var medianFilteredY = _medianFilterY.calculate(-Y.getAsDouble());

    // Apply a linear filter to the median filtered X and Y
    var linearFilteredX = _linearFilterX.calculate(medianFilteredX);
    var linearFilteredY = _linearFilterY.calculate(medianFilteredY);

    // Convert inputs to MPS
    var inputXMPS = linearFilteredX * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
    var inputYMPS = linearFilteredY * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
    var inputRotationRadiansPS = -Z.getAsDouble() * SwerveMap.Chassis.MaxAngularSpeedRadians; // CCW positive

    // Return the proper chassis speeds based on the control mode
    return robotRelative
        ? ChassisSpeeds.fromRobotRelativeSpeeds(
            inputYMPS,
            inputXMPS,
            inputRotationRadiansPS,
            gyroAngle)
        : ChassisSpeeds.fromFieldRelativeSpeeds(
            inputYMPS,
            inputXMPS,
            inputRotationRadiansPS,
            gyroAngle);
  }
}
