package frc.robot.subsystems.elevator;

public enum ElevatorPosition {
    /** The lowest the elevator can physically go */
    kAbsoluteMinimum,
    /** The position to intake from source */
    kSource,
    /** The position to score in the trough, L1 */
    kTrough,
    /** The position to score on the level just above the trough, L2 */
    kLow,
    /** The position to score on the level two above the trought, L3 */
    kMid,
    /** The position to score on the highest level of the reef, L4 */
    kHigh;

    /**
     * Returns a string corresponding to the value of the enum. Used for filtering named
     * commands used in autonmous.
     * @return
     */
    public String getAsRawName() {
        switch (this) {
            default:
                return "L2";
            case kSource:
                return "Source";
            case kTrough:
                return "Trough";
            case kLow:
                return "L2";
            case kMid:
                return "L3";
            case kHigh:
                return "L4";
        }
    }

    public static ElevatorPosition getFromRawName(String name) {
        switch (name) {
            case "Source":
                return kSource;
            case "Trough":
                return kTrough;
            case "L2":
                return kLow;
            case "L3":
                return kMid;
            case "L4":
                return kHigh;
        }

        return null;
    }
}
