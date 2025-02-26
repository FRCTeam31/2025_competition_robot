package frc.robot.subsystems.elevator;

import org.prime.control.ExtendedPIDConstants;

public class ElevatorMap {
    public static final int leftElevatorMotorCANID = 20;
    public static final int rightElevatorMotorCANID = 21;
    public static final int topLimitSwitchChannel = 0;
    public static final int bottomLimitSwitchChannel = 1;
    public static final int maxPercentOutput = 1;

    public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(0, 0, 0, 0, 0, 0, 0);
    public static final double FeedForwardKg = 0.0;

    // TODO: Measure
    public static final double OutputSprocketDiameterMeters = 0;
    public static final double GearRatio = 0;
}
