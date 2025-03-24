package frc.robot.subsystems.endEffector;

import java.util.Map;

import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.elevator.ElevatorPosition;

public class EndEffectorMap {
    // CAN IDs & Channels
    public static final byte IntakeMotorCanID = 19;
    public static final byte WristMotorCanID = 20;
    public static final byte LimitSwitchDIOChannel = 9;

    // Speeds
    public static final double EjectSpeed = -1;
    public static final double IntakeSpeed = 1;
    public static final double WristMaxOutput = 0.175;

    // Physical Constants
    public static final byte IntakeCurrentLimit = 20;
    public static final byte WristCurrentLimit = 30;
    public static final double WristGearRatio = 20;
    public static final double WristMinAngle = -135;
    public static final double WristMaxAngle = -30;
    public static final double WristMaxManuallyControllableAngle = -25;
    public static final double WristNonVerticalAngleThreshold = -110;

    public static final double LowerElevatorSafetyLimit = 0.15;
    public static final Map<ElevatorPosition, Double> ElevatorHeightWristAngleMap = Map.of(
            ElevatorPosition.kAbsoluteMinimum, 0.0,
            ElevatorPosition.kSource, -42.36,
            ElevatorPosition.kTrough, -92.0,
            ElevatorPosition.kL2, -126.0,
            ElevatorPosition.kL3, -126.0,
            ElevatorPosition.kL4, -121.0);

    public static final InterpolatingDoubleTreeMap ElevatorDangerZoneWristAngleLookup = InterpolatingDoubleTreeMap
            .ofEntries(
                    Map.entry(0.15d, -135.0d),
                    Map.entry(0.14d, -120.0d),
                    Map.entry(0.13d, -115.0d),
                    Map.entry(0.12d, -107.5d),
                    Map.entry(0.11d, -100.0d),
                    Map.entry(0.10d, -90d),
                    Map.entry(0.05d, -60d),
                    Map.entry(0.02d, -20d));

    // PID Constants
    public static final ExtendedPIDConstants WristPID = new ExtendedPIDConstants(0.01, 0, 0);
    public static final double WristTolerance = 3.0;
}
