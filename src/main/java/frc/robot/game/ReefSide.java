package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Represents a pair of branches that are connected by a tag.
 */
public class ReefSide {
    public int AprilTagId;

    public ReefBranch LeftBranch;
    public ReefBranch RightBranch;

    public Alliance Alliance;

    public ReefSide(int tagId, ReefBranch leftBranch, ReefBranch rightBranch) {
        AprilTagId = tagId;
        LeftBranch = leftBranch;
        RightBranch = rightBranch;
    }

    public ReefSide(int tagId, ReefBranch leftBranch, ReefBranch rightBranch, Alliance alliance) {
        this(tagId, leftBranch, rightBranch);
        Alliance = alliance;
    }

    public String getFaceName() {
        return ReefBranch.getName(LeftBranch) + ReefBranch.getName(RightBranch);
    }

    public String getBranchName(ReefBranchSide side) {
        return side == ReefBranchSide.kLeft
                ? ReefBranch.getName(LeftBranch)
                : ReefBranch.getName(RightBranch);
    }
}
