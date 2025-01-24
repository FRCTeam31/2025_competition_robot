package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriverDashboard;

import org.littletonrobotics.junction.Logger;
import org.prime.vision.LimelightInputs;
import org.prime.vision.LimelightPose;

public class VisionSubsystem extends SubsystemBase {
    public class VisionMap {
        public static final String LimelightFrontName = "limelight-front";
        public static final String LimelightRearName = "limelight-rear";
    }

    private LimeLightNT[] _limelights;
    private LimelightInputs[] _limelightInputs;
    private LimelightPose[] _limelightRobotPoses;

    public VisionSubsystem() {
        setName("VisionSubsystem");
        var defaultInstance = NetworkTableInstance.getDefault();

        _limelights = new LimeLightNT[] {
                new LimeLightNT(defaultInstance, VisionMap.LimelightFrontName),
                new LimeLightNT(defaultInstance, VisionMap.LimelightRearName) };
        _limelightRobotPoses = new LimelightPose[] {
                _limelightInputs[0].FieldSpaceRobotPose,
                _limelightInputs[1].FieldSpaceRobotPose };
    }

    /**
     * Gets the inputs for the specified limelight.
     * @param llIndex The index of the limelight to get inputs from.
     */
    public LimelightInputs getLimelightInputs(int llIndex) {
        return _limelightInputs[llIndex];
    }

    /**
     * Gets all limelight inputs
     */
    public LimelightInputs[] getAllLimelightInputs() {
        return _limelightInputs;
    }

    /**
     * Gets all limelight inputs
     */
    public LimelightPose[] getAllFieldRobotPoses() {
        return _limelightRobotPoses;
    }

    /**
     * Sets limelight’s LED state.
     *    0 = use the LED Mode set in the current pipeline.
     *    1 = force off.
     *    2 = force blink.
     *    3 = force on.
     * @param llIndex The index of the desired limelight
     * @param mode The LED mode to set
     */
    public void setLedMode(int llIndex, int mode) {
        _limelights[llIndex].setLedMode(mode);
    }

    /**
     * Forces the LED to blink a specified number of times, then returns to pipeline control.
     * @param llIndex The index of the desired limelight
     * @param blinkCount The number of times to blink the LED
     */
    public void blinkLed(int llIndex, int blinkCount) {
        _limelights[llIndex].blinkLed(blinkCount);
    }

    /**
     * Sets limelight’s operation mode.
     *    0 = Vision processor.
     *    1 = Driver Camera (Increases exposure, disables vision processing).
     * @param llIndex The index of the desired limelight
     * @param mode The camera mode to set
     */
    public void setCameraMode(int llIndex, int mode) {
        _limelights[llIndex].setCameraMode(mode);
    }

    /**
     * Sets limelight’s active vision pipeline.
     * @param llIndex The index of the desired limelight
     * @param pipeline The pipeline to set active
     */
    public void setPipeline(int llIndex, int pipeline) {
        _limelights[llIndex].setPipeline(pipeline);
    }

    /**
     * Sets limelight’s streaming mode.
     *    0 = Standard - Side-by-side streams if a webcam is attached to Limelight
     *    1 = PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
     *    2 = PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
     * @param llIndex The index of the desired limelight
     * @param mode The streaming mode to set
     */
    public void setPiPStreamingMode(int llIndex, int mode) {
        _limelights[llIndex].setPiPStreamingMode(mode);
    }

    /**
     * Takes an instantaneous snapshot of the limelight's camera feed.
     * @param llIndex The index of the desired limelight
     */
    public void takeSnapshot(int llIndex) {
        _limelights[llIndex].takeSnapshot();
    }

    /**
     * Set the camera's pose in the coordinate system of the robot.
     * @param llIndex The index of the desired limelight
     * @param pose The Camera's pose to set in Robot space
     */
    public void setCameraPose(int llIndex, Pose3d pose) {
        _limelights[llIndex].setCameraPose(pose);
    }

    public void periodic() {
        // Update all limelight inputs
        for (int i = 0; i < _limelights.length; i++) {
            _limelights[i].updateInputs(_limelightInputs[i]);
            Logger.processInputs("Vision/LL" + i + "/TagID", _limelightInputs[i]);
        }

        // Update Dashboard & logging
        var frontInputs = getLimelightInputs(0);
        var rearInputs = getLimelightInputs(1);
        SmartDashboard.putBoolean("Drive/Vision/Front/IsValidTarget", isAprilTagIdValid(frontInputs.ApriltagId));
        SmartDashboard.putBoolean("Drive/Vision/Rear/IsValidTarget", isAprilTagIdValid(rearInputs.ApriltagId));
        DriverDashboard.FrontApTagIdField.setDouble(frontInputs.ApriltagId);
        DriverDashboard.RearApTagIdField.setDouble(rearInputs.ApriltagId);
        DriverDashboard.RearApTagOffsetDial
                .setDouble(rearInputs.TargetHorizontalOffset.getDegrees());
    }

    public static boolean isAprilTagIdValid(int apriltagId) {
        return apriltagId >= 1 && apriltagId <= 16;
    }
}
