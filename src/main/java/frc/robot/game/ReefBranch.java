package frc.robot.game;

public enum ReefBranch {
    kA,
    kB,
    kC,
    kD,
    kE,
    kF,
    kG,
    kH,
    kI,
    kJ,
    kK,
    kL;

    public static String getBranchName(ReefBranch branch) {
        switch (branch) {
            case kA:
                return "A";
            case kB:
                return "B";
            case kC:
                return "C";
            case kD:
                return "D";
            case kE:
                return "E";
            case kF:
                return "F";
            case kG:
                return "G";
            case kH:
                return "H";
            case kI:
                return "I";
            case kJ:
                return "J";
            case kK:
                return "K";
            case kL:
                return "L";
            default:
                return "Unknown";
        }
    }
}
