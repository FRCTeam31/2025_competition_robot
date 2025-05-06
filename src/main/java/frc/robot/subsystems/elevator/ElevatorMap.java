package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import org.prime.control.ExtendedPIDConstants;
import org.prime.control.MRSGConstants;

import edu.wpi.first.units.Units;

public class ElevatorMap {
        // CAN ids
        public static final int LeftElevatorMotorCANID = 15;
        public static final int RightElevatorMotorCANID = 16;
        public static final int ElevatorEncoderCANID = 22;

        // Limit switch constants
        public static final int TopLimitSwitchChannel = 0;
        public static final int BottomLimitSwitchChannel = 1;
        public static final double BottomLimitResetDebounceSeconds = 0.25;

        // Physical constraints
        public static final double MaxElevatorHeight = 0.63;
        public static final double MaxSpeedCoefficient = 0.5;
        public static final double OutputSprocketDiameterMeters = Units.Millimeters.of(32.2).in(Meters);
        public static final double GearRatio = 16;

        // Manual control 
        public static final int MaxPercentOutput = 1;
        public static final double ManualSpeedLimitAbsolute = 0.5;

        // PIDF constants
        public static final double FeedForwardKg = 0.16733;
        public static final ExtendedPIDConstants PositionPID = new ExtendedPIDConstants(
                        26,
                        0,
                        1.6,
                        0,
                        3.9,
                        0.3,
                        0.355);
}
