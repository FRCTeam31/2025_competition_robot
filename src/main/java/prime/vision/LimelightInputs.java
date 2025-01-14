package prime.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.struct.StructSerializable;

@Logged
public class LimelightInputs implements StructSerializable {

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
     * Target Area (0% of image to 100% of image)
     */
    public double TargetArea = 0.0;
    
    /**
     * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
     */
    public int PipelineLatencyMs = 1;

    /**
     * Time between the end of the exposure of the middle row of the sensor to 
     * the beginning of the tracking pipeline.
     */
    public int CapturePipelineLatencyMs = 2;

    /**
     * The total latency of the capture and pipeline processing in milliseconds.
     */
    public int TotalLatencyMs = 3;

    /**
     * ID of the primary in-view AprilTag
     */
    public int ApriltagId = -1;

    /**
     * Returns the number of AprilTags in the image.
     */
    public double TagCount = 1.1;

    /**
     * Robot transform in field-space.
     */
    public LimelightPose FieldSpaceRobotPose = new LimelightPose(
        new Pose3d(1, 2, 3, new Rotation3d(Units.Degrees.of(1), Units.Degrees.of(2), Units.Degrees.of(3))),
        new double[] { 1, 1, 2.5, 3, 4 });

    /**
     * Robot transform in field-space (alliance driverstation WPILIB origin).
     */
    public LimelightPose RedAllianceOriginFieldSpaceRobotPose = new LimelightPose();

    /**
     * Robot transform in field-space (alliance driverstation WPILIB origin).
     */
    public LimelightPose BlueAllianceOriginFieldSpaceRobotPose = new LimelightPose();

    /**
     * 3D transform of the robot in the coordinate system of the primary in-view AprilTag
     */
    public LimelightPose TargetSpaceRobotPose = new LimelightPose();

    /**
     * 3D transform of the camera in the coordinate system of the primary in-view AprilTag
     */
    public LimelightPose TargetSpaceCameraPose = new LimelightPose();

    /**
     * 3D transform of the camera in the coordinate system of the robot
     */
    public LimelightPose RobotSpaceCameraPose = new LimelightPose();

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Camera
     */
    public LimelightPose CameraSpaceTargetPose = new LimelightPose();

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Robot
     */
    public LimelightPose RobotSpaceTargetPose = new LimelightPose();

    /** Struct for serialization. */
    public static final LimelightInputsStruct struct = new LimelightInputsStruct();
}
