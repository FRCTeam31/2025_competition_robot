package org.prime.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtil {
    /**
    * Optimizes the module angle & drive inversion to ensure the module takes the
    * shortest path to drive at the desired angle
    * 
    * @param desiredState The desired state of the module
    * @param currentAngle The current angle of the module
    */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(-desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return desiredState;
        }
    }
}
