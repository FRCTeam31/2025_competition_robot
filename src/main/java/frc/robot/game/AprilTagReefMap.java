package frc.robot.game;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagReefMap {
    public static final Map<Integer, BranchPair> Branches = Map.ofEntries(
            // Red Alliance
            Map.entry(6, new BranchPair(6, ReefBranch.kK, ReefBranch.kL, Alliance.Red)),
            Map.entry(7, new BranchPair(7, ReefBranch.kA, ReefBranch.kB, Alliance.Red)),
            Map.entry(8, new BranchPair(8, ReefBranch.kC, ReefBranch.kD, Alliance.Red)),
            Map.entry(9, new BranchPair(9, ReefBranch.kE, ReefBranch.kF, Alliance.Red)),
            Map.entry(10, new BranchPair(10, ReefBranch.kH, ReefBranch.kG, Alliance.Red)),
            Map.entry(11, new BranchPair(11, ReefBranch.kI, ReefBranch.kJ, Alliance.Red)),
            // Blue alliance
            Map.entry(17, new BranchPair(17, ReefBranch.kC, ReefBranch.kD, Alliance.Blue)),
            Map.entry(18, new BranchPair(18, ReefBranch.kA, ReefBranch.kB, Alliance.Blue)),
            Map.entry(19, new BranchPair(19, ReefBranch.kK, ReefBranch.kL, Alliance.Blue)),
            Map.entry(20, new BranchPair(20, ReefBranch.kI, ReefBranch.kJ, Alliance.Blue)),
            Map.entry(21, new BranchPair(21, ReefBranch.kH, ReefBranch.kG, Alliance.Blue)),
            Map.entry(22, new BranchPair(22, ReefBranch.kE, ReefBranch.kF, Alliance.Blue)));

    public static BranchPair getBranchPair(int tagId) {
        return Branches.get(tagId);
    }
}
