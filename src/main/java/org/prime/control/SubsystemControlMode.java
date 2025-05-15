package org.prime.control;

public enum SubsystemControlMode {
    /** The subsystem is controlled by the user through a joystick or other input device. */
    ManuallyControlled,
    /** The subsystem is controlled by a PID controller. */
    ClosedLoopControlled,
    /** The subsystem is controlled by the robot's superstructure or another subsystem. */
    SuperStructureControlled,
    /** The subsystem is controlled directly by an autonomous routine. */
    AutoControlled;

    public static SubsystemControlMode getFromRawName(String name) {
        switch (name.toLowerCase()) {
            case "manuallycontrolled":
                return ManuallyControlled;
            case "pidcontrolled":
                return ClosedLoopControlled;
            case "superstructurecontrolled":
                return SuperStructureControlled;
            case "autocontrolled":
                return AutoControlled;
        }

        return null;
    }

    public String getAsRawName() {
        switch (this) {
            default:
                return "ManuallyControlled";
            case ManuallyControlled:
                return "ManuallyControlled";
            case ClosedLoopControlled:
                return "PIDControlled";
            case SuperStructureControlled:
                return "SuperStructureControlled";
            case AutoControlled:
                return "AutoControlled";
        }
    }
}
