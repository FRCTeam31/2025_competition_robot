package prime.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Credit to @Respawn-Robotics for the functions in this class.
 * See their version here: https://github.com/Respawn-Robotics/Swiss-Cheese/blob/master/src/main/java/frc/lib/math/Conversions.java
 */
public class CTREConverter {

  private static final double CANCoderTicks = 4096;
  private static final double FalconEncoderTicks = 2048;

  /**
   * @param positionCounts CANCoder Position Counts
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
    return positionCounts * (360.0 / (gearRatio * CANCoderTicks));
  }

  /**
   * @param positionCounts CANCoder Position Counts
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return Rotation of Mechanism
   */
  public static Rotation2d CANcoderToRotation(double positionCounts, double gearRatio) {
    return Rotation2d.fromDegrees(CANcoderToDegrees(positionCounts, gearRatio));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return CANCoder Position Counts
   */
  public static double degreesToCANcoder(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * CANCoderTicks));
  }

  /**
   * @param rotation Rotation of Mechanism
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return CANCoder Position Counts
   */
  public static double rotationToCANcoder(Rotation2d rotation, double gearRatio) {
    return degreesToCANcoder(rotation.getDegrees(), gearRatio);
  }

  /**
   * @param positionCounts Falcon Position Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Falcon
   */
  public static double falconTicksToDegrees(double positionCounts, double gearRatio) {
    return positionCounts * (360.0 / (gearRatio * FalconEncoderTicks));
  }

  /**
   * @param positionCounts Falcon Position Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Rotation of Falcon
   */
  public static Rotation2d falconTicksToRotation(double positionCounts, double gearRatio) {
    return Rotation2d.fromDegrees(falconTicksToDegrees(positionCounts, gearRatio));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Position Counts
   */
  public static double degreesToFalconTicks(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * FalconEncoderTicks));
  }

  /**
   * @param degrees Rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Position Counts
   */
  public static double rotationToFalconTicks(Rotation2d rotation, double gearRatio) {
    return degreesToFalconTicks(rotation.getDegrees(), gearRatio);
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconTicksToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / FalconEncoderTicks);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double RPMToFalconTicks(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (FalconEncoderTicks / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
   * @return Falcon Velocity Counts
   */
  public static double falconTicksToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconTicksToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalconTicks(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalconTicks(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  /**
   * @param positionCounts Falcon Position Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Wheel
   * @return Meters
   */
  public static double falconTicksToMeters(double positionCounts, double circumference, double gearRatio) {
    return positionCounts * (circumference / (gearRatio * FalconEncoderTicks));
  }

  /**
   * @param meters Meters
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Wheel
   * @return Falcon Position Counts
   */
  public static double metersToFalconTicks(double meters, double circumference, double gearRatio) {
    return meters / (circumference / (gearRatio * FalconEncoderTicks));
  }

  /**
   * @param rotations rotations
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio of mechanism
   * @return
   */
  public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
    return rotations * (circumference / gearRatio);
  }

  /**
   * @param meters meters
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio of mechanism
   */
  public static double metersToRotations(double meters, double circumference, double gearRatio) {
    return meters / (circumference / gearRatio);
  }
}
