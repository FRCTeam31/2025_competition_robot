package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.helpers.LimelightResults;
import frc.robot.subsystems.vision.helpers.PoseEstimate;

@AutoLog
public class LimelightInputs {

    /**
     * Horizontal Offset From Crosshair To Target 
     * (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    public Rotation2d TargetHorizontalOffset = new Rotation2d(Math.PI);

    /**
     * Vertical Offset From Crosshair To Target 
     * (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    public Rotation2d TargetVerticalOffset = new Rotation2d();

    /**
     * The robot's MT1 pose in field space.
     */
    public PoseEstimate BotPoseEstimate = new PoseEstimate();

    /**
     * The robot's MT2 pose in field space.
     */
    public PoseEstimate BotPoseEstimateMT2 = new PoseEstimate();

    /**
     * The JSON dump from this limelight.
     */
    public LimelightResults CurrentResults = new LimelightResults();
}
