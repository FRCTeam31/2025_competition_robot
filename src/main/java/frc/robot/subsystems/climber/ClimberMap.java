package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberMap {
    // CAN IDs and channels
    public static final byte ClimberLeftMotorCANID = 17;
    public static final byte ClimberRightMotorCANID = 18;
    public static final byte ClimberHookMotorCANID = 21;
    public static final byte HooksClosedLimitSwitchChannel = 2;
    public static final byte HooksOpenLimitSwitchChannel = 4;
    public static final byte ClimberOutLimitSwitchChannel = 5;
    public static final byte ClimberInLimitSwitchChannel = 6;

    // Physical constants
    public static final byte ClimberGearRatio = 100;
    public static final double ClimbingPitchThresholdDegrees = 5;
    public static final double ClimberArmlengthMeters = Units.inchesToMeters(16);
    public static final double ClimberOutputShaftDiameterMeters = Units.inchesToMeters(0.5);
    public static final double FullyClimbedOutputRotations = 2.8;

    // Speeds and timeouts
    public static final double WinchInSpeed = 1;
    public static final double HookCloseSpeed = 1;
    public static final double ClimberChangeStateTimeout = 7;
    public static final double HooksChangeStateTimeout = 5;
    public static final double ClimbAngleResetDebounceSeconds = 0.25;
}
