package org.prime.util;

public class MotorUtil {
    /**
     * Returns the speed of the motor with the limit switches applied. 
     * If the limit is reached and the motor is trying to move in that direction, the speed is limited to 0 in the direction of the limit.
     * A positive speed is assumed to be moving towards the forward limit, and a negative speed is assumed to be moving towards the reverse limit.
     * @param intendedSpeed
     * @param fwdLimitReached
     * @param revLimitReached
     * @return
     */
    public static double getMotorspeedWithLimits(double intendedSpeed, boolean fwdLimitReached,
            boolean revLimitReached) {
        if (fwdLimitReached) {
            return Math.min(intendedSpeed, 0);
        }

        if (revLimitReached) {
            return Math.max(intendedSpeed, 0);
        }

        return intendedSpeed;
    }
}
