package frc.robot.subsystems.elevator;

public enum ElevatorPosition {
    /** The lowest the elevator can physically go */
    kAbsoluteMinimum,
    /** The position to intake from source */
    kSource,
    /** The position to score in the trough, L1 */
    kTrough,
    /** The position to score on the level just above the trough, L2 */
    kL2,
    /** The position to score on the level two above the trought, L3 */
    kL3,
    /** The position to score on the highest level of the reef, L4 */
    kL4;

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
            case kL2:
                return "L2";
            case kL3:
                return "L3";
            case kL4:
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
                return kL2;
            case "L3":
                return kL3;
            case "L4":
                return kL4;
        }

        return null;
    }
}
