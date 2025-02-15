package org.prime.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.SwerveMap;

public class SwerveControlSuppliers {

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

    // Convert inputs to MPS
    var inputXMPS = X.getAsDouble() * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
    var inputYMPS = -Y.getAsDouble() * SwerveMap.Chassis.MaxSpeedMetersPerSecond;
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
