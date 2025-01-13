package frc.robot.subsystems.vision;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import prime.vision.LimelightInputs;
import prime.vision.LimelightPose;

public class LimeLightNT implements AutoCloseable {
    private NetworkTable m_limelightTable;
    private ExecutorService m_executorService = Executors.newSingleThreadExecutor();

    private LimelightInputs m_inputs = new LimelightInputs();

    public LimeLightNT(NetworkTableInstance instance, String tableName) {
        m_limelightTable = instance.getTable(tableName);
    }

    public LimelightInputs getInputs() {
        m_inputs.TargetHorizontalOffset = getHorizontalOffsetFromTarget();
        m_inputs.TargetVerticalOffset = getVerticalOffsetFromTarget();
        m_inputs.TargetArea = getTargetArea();
        m_inputs.PipelineLatencyMs = getPipelineLatencyMs();
        m_inputs.CapturePipelineLatencyMs = getCapturePipelineLatencyMs();
        m_inputs.TotalLatencyMs = getTotalLatencyMs();
        m_inputs.ApriltagId = getApriltagId();
        m_inputs.TagCount = getTagCount();
        m_inputs.FieldSpaceRobotPose = getRobotPose();
        m_inputs.RedAllianceOriginFieldSpaceRobotPose = getRobotPose(Alliance.Red);
        m_inputs.BlueAllianceOriginFieldSpaceRobotPose = getRobotPose(Alliance.Blue);

        return m_inputs;
    }

    //#region Basic Targeting Data

    /**
     * Returns Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    public Rotation2d getHorizontalOffsetFromTarget() {
        return Rotation2d.fromDegrees(m_limelightTable.getEntry("tx").getDouble(0.0));
    }

    /**
     * Returns Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    public Rotation2d getVerticalOffsetFromTarget() {
        return Rotation2d.fromDegrees(m_limelightTable.getEntry("ty").getDouble(0.0));
    }

    /**
     * Returns Target Area (0% of image to 100% of image)
     */
    public double getTargetArea() {
        return m_limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
     */
    public int getPipelineLatencyMs() {
        return m_limelightTable.getEntry("tl").getNumber(0).intValue();
    }

    /**
     * Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
     */
    public int getCapturePipelineLatencyMs() {
        return m_limelightTable.getEntry("cl").getNumber(0).intValue();
    }

    /**
     * The total latency of the capture and pipeline processing in milliseconds.
     * @return
     */
    public int getTotalLatencyMs() {
        return getPipelineLatencyMs() + getCapturePipelineLatencyMs();
    }

    //#endregion

    //#region AprilTag and 3D Data

    /**
     * ID of the primary in-view AprilTag
     */
    public int getApriltagId() {
        return m_limelightTable.getEntry("tid").getNumber(0).intValue();
    }

    /**
     * Returns the number of AprilTags in the image.
     * @return
     */
    public double getTagCount() {
        // Robot transform in field-space. Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
        var botPose = m_limelightTable.getEntry("botpose").getDoubleArray(new double[11]);

        return botPose[7];
    }

    /**
     * Robot transform in field-space.
     */
    public LimelightPose getRobotPose() {
        var poseData = m_limelightTable.getEntry("botpose").getDoubleArray(new double[11]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

        return new LimelightPose(poseData, calculateTrust(poseData[7]));
    }

    /**
     * Robot transform in field-space (alliance driverstation WPILIB origin).
     * @param alliance
     */
    public LimelightPose getRobotPose(DriverStation.Alliance alliance) {
        var poseData = alliance == Alliance.Blue
        ? m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11])
        : m_limelightTable.getEntry("botpose_wpired").getDoubleArray(new double[11]);

        return new LimelightPose(poseData, calculateTrust(poseData[7]));
    }

    /**
     * 3D transform of the robot in the coordinate system of the primary in-view AprilTag
     */
    public LimelightPose getRobotPoseInTargetSpace() {
        var poseData = m_limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[11]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

        return new LimelightPose(poseData, calculateTrust(poseData[7]));
    }

    /**
     * 3D transform of the camera in the coordinate system of the primary in-view AprilTag
     */
    public LimelightPose getCameraPoseInTargetSpace() {
        var poseData = m_limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[11]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

        return new LimelightPose(poseData, calculateTrust(poseData[7]));
    }

    /**
     * 3D transform of the camera in the coordinate system of the robot
     */
    public LimelightPose getCameraPoseInRobotSpace() {
        var poseData = m_limelightTable.getEntry("camerapose_robotspace").getDoubleArray(new double[11]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

        return new LimelightPose(poseData, calculateTrust(poseData[7]));
    }

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Camera
     */
    public LimelightPose getTargetPoseInCameraSpace() {
        var poseData = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[11]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

        return new LimelightPose(poseData, calculateTrust(poseData[7]));
    }

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Robot
     */
    public LimelightPose getTargetPoseInRobotSpace() {
        var poseData = m_limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[11]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

        return new LimelightPose(poseData, calculateTrust(poseData[7]));
    }

    /**
     * Calculates a trust value based on the number of tags in view.
     * @return
     */
    public double[] calculateTrust(double tagCount) {
        // Trust level is a function of the number of tags in view
        // var trustLevel = 0.490956d + Math.pow(9998.51d, -(6.95795d * tagCount));

        var trustLevel = tagCount >= 2.0 ? 2 : 20;

        // X Y Z trust levels (never trust Z)
        return VecBuilder.fill(trustLevel, trustLevel, 999999).getData();
    }

    //#endregion

    //#region Camera Controls

    /**
     * Sets limelight’s LED state.
     *    0 = use the LED Mode set in the current pipeline.
     *    1 = force off.
     *    2 = force blink.
     *    3 = force on.
     * @param mode
     */
    public void setLedMode(int mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Forces the LED to blink a specified number of times, then returns to pipeline control.
     */
    public void blinkLed(int blinkCount) {
        m_executorService.submit(() -> {
        // Blink the LED X times with 100ms on, 200ms off for each blink
        for (int i = 0; i < blinkCount; i++) {
            m_limelightTable.getEntry("ledMode").setNumber(3);

            try {
            Thread.sleep(100);
            } catch (Exception e) {
            Thread.currentThread().interrupt();
            }

            m_limelightTable.getEntry("ledMode").setNumber(1);
            try {
            Thread.sleep(200);
            } catch (Exception e) {
            Thread.currentThread().interrupt();
            }
        }

        // Then return to pipeline control
        setLedMode(0);
        });
    }

    /**
     * Sets limelight’s operation mode.
     *    0 = Vision processor.
     *    1 = Driver Camera (Increases exposure, disables vision processing).
     * @param mode
     */
    public void setCameraMode(int mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode);
    }

    /**
     * Sets limelight’s pipeline.
     * @param pipeline
     */
    public void setPipeline(int pipeline) {
        m_limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Side-by-side streams (Note: USB output stream is not affected by this mode)
     * @param mode
     */
    public void setPiPStreamingMode(int mode) {
        m_limelightTable.getEntry("stream").setNumber(mode);
    }

    /**
     * Allows users to take snapshots during a match
     */
    public void takeSnapshot() {
        m_limelightTable.getEntry("snapshot").setNumber(0); // Reset
        m_limelightTable.getEntry("snapshot").setNumber(1); // Take snapshot
        m_limelightTable.getEntry("snapshot").setNumber(0); // Reset
    }

    /**
     * Set the camera's pose in the coordinate system of the robot.
     * @param pose
     */
    public void setCameraPose(Pose3d pose) {
        var poseData = new double[] {
        pose.getTranslation().getX(),
        pose.getTranslation().getY(),
        pose.getTranslation().getZ(),
        Units.radiansToDegrees(pose.getRotation().getX()),
        Units.radiansToDegrees(pose.getRotation().getY()),
        Units.radiansToDegrees(pose.getRotation().getZ()),
        };

        m_limelightTable.getEntry("camerapose_robotspace_set").setDoubleArray(poseData);
    }

    //#endregion

    public void close() {
        m_executorService.shutdown();
    }
}
