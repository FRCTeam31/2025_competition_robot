package frc.robot.subsystems.endEffector;

import org.prime.control.ExtendedPIDConstants;

public class EndEffectorMap {
    public static final byte IntakeMotorCanID = 127;
    public static final byte WristMotorCanID = 127;
    public static final byte LimitSwitchCanID = 0;
    public static final double EjectSpeed = 0.5;
    public static final double IntakeSpeed = -0.5;

    public static final ExtendedPIDConstants WristPID = new ExtendedPIDConstants(0, 0, 0);

    public static final double GearRatio = 1;

}
