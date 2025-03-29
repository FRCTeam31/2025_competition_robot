package frc.robot.subsystems.swerve.util;

import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveMap;

/**
 * A class that uses a PID controller to calculate a rotational correction speed for the robot to face a setpoint angle
 */
public class AutoAlign {
    private Rotation2d _setpoint = Rotation2d.fromDegrees(0);
    private PIDController _pid;

    public AutoAlign(ExtendedPIDConstants pidConstants) {
        _pid = pidConstants.createPIDController(0.02);
        _pid.enableContinuousInput(-Math.PI, Math.PI);
        _pid.setTolerance(Math.PI / 180d);
    }

    /**
    * Sets the snap-to gyro setpoint, converting from degrees to radians
    * 
    * @param angle The angle to snap to in degrees
    */
    public void setSetpoint(Rotation2d angle) {
        var setpointModulated = MathUtil.angleModulus(angle.getRadians());
        _setpoint = Rotation2d.fromRadians(setpointModulated);
    }

    /**
    * Calculates the snap angle correction using the PID controller
    * 
    * @param currentAngle The current angle of the robot
    * @return The rotational speed correction 
    */
    public double getCorrection(Rotation2d currentAngle) {
        var currentRotationRadians = MathUtil.angleModulus(currentAngle.getRadians());
        var correction = _pid.calculate(currentRotationRadians, _setpoint.getRadians());

        correction = MathUtil.applyDeadband(correction, 0.01);

        return MathUtil.clamp(correction, -SwerveMap.Chassis.MaxAngularSpeedRadians,
                SwerveMap.Chassis.MaxAngularSpeedRadians);
    }

    /**
     * Gets the current setpoint
     */
    public Rotation2d getSetpoint() {
        return _setpoint;
    }

    /**
     * Checks if the PID controller is at the setpoint
     */
    public boolean atSetpoint() {
        return _pid.atSetpoint();
    }

    /**
     * Resets the PID controller
     */
    public void resetPID() {
        _pid.reset();
    }
}
