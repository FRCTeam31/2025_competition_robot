package org.prime.pose;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class PoseUtil {
        public static Trajectory generateStraightLineTrajectory(Pose2d startPose, double distanceMeters,
                        TrajectoryConfig config) {
                // Extract heading from pose
                Rotation2d heading = startPose.getRotation();

                // Compute end position
                Translation2d endTranslation = new Translation2d(
                                startPose.getX() + distanceMeters * heading.getCos(),
                                startPose.getY() + distanceMeters * heading.getSin());

                Pose2d endPose = new Pose2d(endTranslation, heading);

                // Generate trajectory
                return TrajectoryGenerator.generateTrajectory(
                                List.of(startPose, endPose),
                                config);
        }

        /**
         * Gets the poses from a trajectory.
         * @param trajectory
         * @return
         */
        public static List<Pose2d> getTrajectoryPoses(Trajectory trajectory) {
                var states = trajectory.getStates();
                var poses = new ArrayList<Pose2d>(states.size());
                for (var state : states) {
                        poses.add(state.poseMeters);
                }

                return poses;
        }

        /**
         * Gets the distance between two poses.
         * @param pose1
         * @param pose2
         * @return
         */
        public static double getDistanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
                return pose1.getTranslation().getDistance(pose2.getTranslation());
        }

        /**
         * Returns the closest pose in the list to the current pose, or null if the list is empty.
         * @param currentPose
         * @param poses
         * @return
         */
        public static Pose2d getClosestPoseInList(Pose2d currentPose, List<Pose2d> poses) {
                return poses.stream()
                                .min((pose1, pose2) -> Double.compare(
                                                getDistanceBetweenPoses(currentPose, pose1),
                                                getDistanceBetweenPoses(currentPose, pose2)))
                                .orElse(null);
        }

        /**
         * Converts a pose from robot space to field space.
         * @param robotPose_Field The robot's pose in field space.
         * @param targetPose_Robot The target pose in robot space.
         * @return
         */
        public static Pose2d convertPoseFromRobotToFieldSpace(Pose2d robotPose_Field, Pose2d targetPose_Robot) {
                // Transform target translation from robot space to field space
                Translation2d targetTranslation_Field = robotPose_Field.getTranslation()
                                .plus(targetPose_Robot.getTranslation().rotateBy(robotPose_Field.getRotation()));

                // Compute new field-space rotation
                Rotation2d targetRotation_Field = robotPose_Field.getRotation()
                                .plus(targetPose_Robot.getRotation());

                return new Pose2d(targetTranslation_Field, targetRotation_Field);
        }
}
