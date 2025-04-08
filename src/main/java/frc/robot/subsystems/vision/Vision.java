package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.prime.vision.LimelightInputs;

public class Vision extends SubsystemBase {
    public class VisionMap {
        public static final String LimelightFrontName = "limelight-front";
        public static final String LimelightRearName = "limelight-rear";
        public static final double[] ReefAprilTags = new double[] {
                6, 7, 8, 9, 10, 11,
                17, 18, 19, 20, 21, 22
        };
    }

    Map<LimelightNameEnum, LimeLightNT> _limelights = new HashMap<>();
    Map<LimelightNameEnum, LimelightInputs> _limelightInputs = new HashMap<>();

    public Vision() {
        setName("Vision");
        var defaultInstance = NetworkTableInstance.getDefault();

        _limelights.put(LimelightNameEnum.kFront, new LimeLightNT(defaultInstance, VisionMap.LimelightFrontName));
        _limelights.put(LimelightNameEnum.kRear, new LimeLightNT(defaultInstance, VisionMap.LimelightRearName));

        _limelightInputs.put(LimelightNameEnum.kFront, new LimelightInputs());
        _limelightInputs.put(LimelightNameEnum.kRear, new LimelightInputs());
    }

    /**
     * Gets the inputs for the specified limelight.
     * @param llIndex The index of the limelight to get inputs from.
     */
    public LimelightInputs getLimelightInputs(LimelightNameEnum ll) {
        return _limelightInputs.get(ll);
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
    public void setLedMode(LimelightNameEnum ll, int mode) {
        _limelights.get(ll).setLedMode(mode);
    }

    /**
     * Forces the LED to blink a specified number of times, then returns to pipeline control.
     * @param llIndex The index of the desired limelight
     * @param blinkCount The number of times to blink the LED
     */
    public void blinkLed(LimelightNameEnum ll, int blinkCount) {
        _limelights.get(ll).blinkLed(blinkCount);
    }

    /**
     * Sets limelight’s active vision pipeline.
     * @param llIndex The index of the desired limelight
     * @param pipeline The pipeline to set active
     */
    public void setPipeline(LimelightNameEnum ll, int pipeline) {
        _limelights.get(ll).setPipeline(pipeline);
    }

    /**
     * Sets limelight’s streaming mode.
     *    0 = Standard - Side-by-side streams if a webcam is attached to Limelight
     *    1 = PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
     *    2 = PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
     * @param llIndex The index of the desired limelight
     * @param mode The streaming mode to set
     */
    public void setPiPStreamingMode(LimelightNameEnum ll, int mode) {
        _limelights.get(ll).setPiPStreamingMode(mode);
    }

    /**
     * Set the camera's pose in the coordinate system of the robot.
     * @param llIndex The index of the desired limelight
     * @param pose The Camera's pose to set in Robot space
     */
    public void setCameraPose(LimelightNameEnum ll, Pose3d pose) {
        _limelights.get(ll).setCameraPose(pose);
    }

    @Override
    public void periodic() {
        // Update all limelight inputs
        for (var ll : _limelights.keySet()) {
            _limelights.get(ll).updateInputs(_limelightInputs.get(ll));
            Logger.processInputs("Vision/LL/" + ll.name(), _limelightInputs.get(ll));
        }
    }

    public static boolean isAprilTagIdValid(int apriltagId) {
        return apriltagId >= 1 && apriltagId <= 22;
    }

    public static boolean isReefTag(int tagId) {
        for (var i = 0; i < VisionMap.ReefAprilTags.length; i++) {
            if (tagId == VisionMap.ReefAprilTags[i])
                return true;
        }

        return false;
    }

    //#region Commands

    public Command setLimelightPipeline(LimelightNameEnum ll, int pipeline) {
        return Commands.runOnce(() -> setPipeline(ll, pipeline));
    }

    //#endregion
}
