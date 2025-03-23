package org.prime.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Container;
import frc.robot.subsystems.drivetrain.SwerveMap;

public class SwerveControlSuppliers {

  public DoubleSupplier X;
  public DoubleSupplier Y;
  public DoubleSupplier Z;

  // The number of samples for which the input filters operate over
  private boolean _useFiltering = false;
  private static final int _sampleWindow = 5;
  private MedianFilter _medianFilterX = new MedianFilter(_sampleWindow * 2);
  private MedianFilter _medianFilterY = new MedianFilter(_sampleWindow * 2);
  private MedianFilter _medianFilterZ = new MedianFilter(_sampleWindow * 2);
  private LinearFilter _linearFilterX = LinearFilter.movingAverage(_sampleWindow);
  private LinearFilter _linearFilterY = LinearFilter.movingAverage(_sampleWindow);
  private LinearFilter _linearFilterZ = LinearFilter.movingAverage(_sampleWindow);

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

    var x = X.getAsDouble();
    var y = -Y.getAsDouble();
    var z = -Z.getAsDouble();

    // var scaledX = x / (Container.Elevator.getElevatorPositionPercent() + 0.5);
    // var scaledY = y / (Container.Elevator.getElevatorPositionPercent() + 0.5);
    // var scaledZ = z / (Container.Elevator.getElevatorPositionPercent() + 0.5);

    // var finalX = MathUtil.clamp(scaledX, 0.4 * x, x);

    return _useFiltering
        ? getFilteredChassisSpeeds(x, y, z, gyroAngle, robotRelative)
        : getRawChassisSpeeds(x, y, z, gyroAngle, robotRelative);
  }

  private ChassisSpeeds getFilteredChassisSpeeds(double rawX, double rawY, double rawZ, Rotation2d gyroAngle,
      boolean robotRelative) {
    // Apply median filter to the raw inputs
    var medianFilteredX = _medianFilterX.calculate(rawX);
    var medianFilteredY = _medianFilterY.calculate(rawY);
    var medianFilteredZ = _medianFilterZ.calculate(rawZ);

    // Apply linear filters to the median filtered inputs
    var linearFilteredX = _linearFilterX.calculate(medianFilteredX);
    var linearFilteredY = _linearFilterY.calculate(medianFilteredY);
    var linearFilteredZ = _linearFilterZ.calculate(medianFilteredZ);

    // Convert filtered inputs to MPS
    var inputXMPS = linearFilteredX * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
    var inputYMPS = linearFilteredY * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
    var inputRotationRadiansPS = linearFilteredZ * SwerveMap.Chassis.MaxAngularSpeedRadians; // CCW positive

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

  private ChassisSpeeds getRawChassisSpeeds(double rawX, double rawY, double rawZ, Rotation2d gyroAngle,
      boolean robotRelative) {
    // Convert inputs to MPS
    var inputXMPS = rawX * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
    var inputYMPS = rawY * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
    var inputRotationRadiansPS = rawZ * SwerveMap.Chassis.MaxAngularSpeedRadians; // CCW positive

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
