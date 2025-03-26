package frc.robot.game;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagReefMap {
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
        return Branches.get(tagId);
    }

    /**
    * Returns the pose of the desired side of reef pegs from the current robot pose (assumed to be the midpoint between pegs)
    */
    public static Pose2d getBranchPoseFromAprilTag(ReefBranchSide side, Pose2d inputPose) {
        // Determine the translation direction (left = +90°, right = -90°)
        var offsetRotation = inputPose.getRotation().plus(side == ReefBranchSide.kLeft
                ? Rotation2d.fromDegrees(90)
                : Rotation2d.fromDegrees(-90));

        // Apply a translation of 16.5 cm in the calculated direction
        var branchTranslation = new Translation2d(Centimeters.mutable(16.5).in(Meters), offsetRotation);

        return new Pose2d(inputPose.getTranslation().plus(branchTranslation), inputPose.getRotation());
    }

    public static Pose2d getBranchApproachPose(
            ReefBranchSide branchSide,
            Pose2d aprilTagPose,
            double approachDistance) {
        // Select the correct reef branch (left or right)
        var selectedBranchPose = getBranchPoseFromAprilTag(branchSide, aprilTagPose);

        // Translate the pose approachDistance meters forward in the direction of its rotation
        var approachTranslation = selectedBranchPose.getTranslation()
                .plus(new Translation2d(approachDistance, selectedBranchPose.getRotation()));

        // The robot should be facing the reef, so use the AprilTag's rotation
        return new Pose2d(approachTranslation, aprilTagPose.getRotation());
    }
}
