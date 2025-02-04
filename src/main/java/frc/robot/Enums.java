package frc.robot;

public class Enums {
    public enum ElevatorLocation {
        /** Starting configuration of the elevator*/
        STARTING,
        /** Absolute lowest place to score coral */
        TROUGH,
        /** The lowest place to score coral above trough */
        LOWER,
        /** The second place to score coral in between Lower and Upper*/
        MID,
        /** The highest place to score coral, vertically*/
        UPPER
    }
}