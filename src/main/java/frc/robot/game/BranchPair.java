package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Represents a pair of branches that are connected by a tag.
 */
public class BranchPair {
    public int AprilTagId;

    public ReefBranch LeftBranch;
    public ReefBranch RightBranch;

    public Alliance Alliance;

    public BranchPair(int tagId, ReefBranch leftBranch, ReefBranch rightBranch) {
        AprilTagId = tagId;
        LeftBranch = leftBranch;
        RightBranch = rightBranch;
    }

    public BranchPair(int tagId, ReefBranch leftBranch, ReefBranch rightBranch, Alliance alliance) {
        this(tagId, leftBranch, rightBranch);
        Alliance = alliance;
    }

    public String getFaceName() {
        return ReefBranch.getBranchName(LeftBranch) + ReefBranch.getBranchName(RightBranch);
    }
}
