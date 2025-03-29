package frc.robot.game;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagReefMap {
    private static AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    static {
        // Ensure the AprilTagFieldLayout is loaded correctly
        if (FieldLayout == null) {
            throw new IllegalStateException("AprilTagFieldLayout could not be loaded.");
        }

        FieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    public static final Map<Integer, ReefSide> Branches = Map.ofEntries(
            // Red Alliance
            Map.entry(6, new ReefSide(6, ReefBranch.kK, ReefBranch.kL, Alliance.Red)),
            Map.entry(7, new ReefSide(7, ReefBranch.kA, ReefBranch.kB, Alliance.Red)),
            Map.entry(8, new ReefSide(8, ReefBranch.kC, ReefBranch.kD, Alliance.Red)),
            Map.entry(9, new ReefSide(9, ReefBranch.kE, ReefBranch.kF, Alliance.Red)),
            Map.entry(10, new ReefSide(10, ReefBranch.kH, ReefBranch.kG, Alliance.Red)),
            Map.entry(11, new ReefSide(11, ReefBranch.kI, ReefBranch.kJ, Alliance.Red)),
            // Blue alliance
            Map.entry(17, new ReefSide(17, ReefBranch.kC, ReefBranch.kD, Alliance.Blue)),
            Map.entry(18, new ReefSide(18, ReefBranch.kA, ReefBranch.kB, Alliance.Blue)),
            Map.entry(19, new ReefSide(19, ReefBranch.kK, ReefBranch.kL, Alliance.Blue)),
            Map.entry(20, new ReefSide(20, ReefBranch.kI, ReefBranch.kJ, Alliance.Blue)),
            Map.entry(21, new ReefSide(21, ReefBranch.kH, ReefBranch.kG, Alliance.Blue)),
            Map.entry(22, new ReefSide(22, ReefBranch.kE, ReefBranch.kF, Alliance.Blue)));

    public static ReefSide getReefSide(int tagId) {
        var reefSide = Branches.get(tagId);
        reefSide.TagPose = FieldLayout.getTagPose(tagId);

        return reefSide;
    }

    /**
    * Returns the pose of the desired side of reef pegs from the current robot pose (assumed to be the midpoint between pegs)
    * @param side The side of the reef pegs to get the pose of
    * @param targetPose The target pose
    */
    public static Pose2d getBranchPoseFromTarget(ReefBranchSide side, Pose2d targetPose) {
        // Determine the translation direction (left = +90°, right = -90°)
        var offsetRotation = targetPose.getRotation().plus(side == ReefBranchSide.kLeft
                ? Rotation2d.fromDegrees(90)
                : Rotation2d.fromDegrees(-90));

        // Apply a translation of 16.5 cm in the calculated direction
        var branchTranslation = new Translation2d(Centimeters.mutable(16.5).in(Meters), offsetRotation);

        return new Pose2d(targetPose.getTranslation().plus(branchTranslation), targetPose.getRotation());
    }

    /**
     * Returns a pose that is a desired distance in the direction the target pose is facing
     * @param branchPose The target pose
     * @param approachDistance The distance of the pose in the direction the target pose is facing
     */
    public static Pose2d getApproachPose(Pose2d branchPose, double approachDistance) {
        // Translate the pose approachDistance meters forward in the direction of its rotation
        var approachTranslation = branchPose.getTranslation()
                .plus(new Translation2d(approachDistance, branchPose.getRotation()));

        // The robot should be facing the reef, so use the AprilTag's rotation
        return new Pose2d(approachTranslation, branchPose.getRotation());
    }
}
