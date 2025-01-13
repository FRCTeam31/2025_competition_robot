package prime.control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

public class PrimeXboxController extends CommandXboxController {

  public PrimeXboxController(int port) {
    super(port);
  }

  /**
   * Returns a cubic-scaled supplier for the left stick's Y axis with a deadband and curve weight
   * @param deadband (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getLeftStickYSupplier(double deadband, double curveWeight) {
    return () -> -Controls.cubicScaledDeadband(getRawAxis(Controls.LEFT_STICK_Y), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the left stick's Y axis with a deadband
   * @param deadband (0-1)
   */
  public DoubleSupplier getLeftStickYSupplier(double deadband) {
    return () -> -Controls.linearScaledDeadband(getRawAxis(Controls.LEFT_STICK_Y), deadband);
  }

  /**
   * Returns a cubic-scaled supplier for the left stick's X axis with a deadband and curve weight
   * @param deadband (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getLeftStickXSupplier(double deadband, double curveWeight) {
    return () -> Controls.cubicScaledDeadband(getRawAxis(Controls.LEFT_STICK_X), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the left stick's X axis with a deadband
   * @param deadband (0-1)
   */
  public DoubleSupplier getLeftStickXSupplier(double deadband) {
    return () -> Controls.linearScaledDeadband(getRawAxis(Controls.LEFT_STICK_X), deadband);
  }

  /**
   * Returns a cubic-scaled supplier for the right stick's Y axis with a deadband and curve weight
   * @param deadband (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getRightStickYSupplier(double deadband, double curveWeight) {
    return () -> -Controls.cubicScaledDeadband(getRawAxis(Controls.RIGHT_STICK_Y), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the right stick's Y axis with a deadband
   * @param deadband (0-1)
   */
  public DoubleSupplier getRightStickYSupplier(double deadband) {
    return () -> -Controls.linearScaledDeadband(getRawAxis(Controls.RIGHT_STICK_Y), deadband);
  }

  /**
   * Returns a cubic-scaled supplier for the right stick's X axis with a deadband and curve weight
   * @param deadband (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getRightStickXSupplier(double deadband, double curveWeight) {
    return () -> Controls.cubicScaledDeadband(getRawAxis(Controls.RIGHT_STICK_X), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the right stick's X axis with a deadband
   * @param deadband (0-1)
   */
  public DoubleSupplier getRightStickXSupplier(double deadband) {
    return () -> Controls.linearScaledDeadband(getRawAxis(Controls.RIGHT_STICK_X), deadband);
  }

  /**
   * Returns a supplier for the trigger axis with the left trigger subtracted from the right trigger
   */
  public DoubleSupplier getTriggerSupplier(double deadband, double curveWeight) {
    return () ->
      Controls.cubicScaledDeadband(getRawAxis(Controls.RIGHT_TRIGGER), deadband, curveWeight) -
      Controls.cubicScaledDeadband(getRawAxis(Controls.LEFT_TRIGGER), deadband, curveWeight);
  }

  /**
   * Returns a supplier for the trigger axis with the right trigger subtracted from the left trigger
   */
  public DoubleSupplier getInvertedTriggerSupplier() {
    return () -> getRawAxis(Controls.LEFT_TRIGGER) - getRawAxis(Controls.RIGHT_TRIGGER);
  }

  public SwerveControlSuppliers getSwerveControlProfile(
    HolonomicControlStyle style,
    double driveDeadband,
    double curveWeight
  ) {
    switch (style) {
      default:
      case Standard:
        return new SwerveControlSuppliers(
          getLeftStickXSupplier(driveDeadband, curveWeight),
          getLeftStickYSupplier(driveDeadband, curveWeight),
          getTriggerSupplier(driveDeadband, curveWeight)
        );
      case Drone:
        return new SwerveControlSuppliers(
          getRightStickXSupplier(driveDeadband, curveWeight),
          getRightStickYSupplier(driveDeadband, curveWeight),
          getLeftStickXSupplier(driveDeadband, curveWeight)
        );
    }
  }
}
