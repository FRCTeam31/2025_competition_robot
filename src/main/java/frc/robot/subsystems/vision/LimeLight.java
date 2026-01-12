package frc.robot.subsystems.vision;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.helpers.LimelightHelpers;

public class LimeLight implements AutoCloseable {
    private String _limelightName;
    private ExecutorService m_executorService = Executors.newSingleThreadExecutor();

    public LimeLight(String limelightName) {
        _limelightName = limelightName;
    }

    public void updateInputs(LimelightInputs inputs) {
        inputs.TargetHorizontalOffset = getHorizontalOffsetFromTarget();
        inputs.TargetVerticalOffset = getVerticalOffsetFromTarget();
        inputs.BotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(_limelightName);
        inputs.BotPoseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);
        inputs.CurrentResults = LimelightHelpers.getLatestResults(_limelightName);
    }

    //#region Basic Targeting Data

    /**
     * Returns Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    public Rotation2d getHorizontalOffsetFromTarget() {
        return Rotation2d.fromDegrees(LimelightHelpers.getTX(_limelightName));
    }

    /**
     * Returns Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    public Rotation2d getVerticalOffsetFromTarget() {
        return Rotation2d.fromDegrees(LimelightHelpers.getTY(_limelightName));
    }

    /**
     * Returns Target Area (0% of image to 100% of image)
     */
    public double getTargetArea() {
        return LimelightHelpers.getTA(_limelightName);
    }

    /**
     * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
     */
    public double getPipelineLatencyMs() {
        return LimelightHelpers.getLatency_Pipeline(_limelightName);
    }

    /**
     * Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
     */
    public double getCapturePipelineLatencyMs() {
        return LimelightHelpers.getLatency_Capture(_limelightName);
    }

    //#endregion

    //#region Camera Controls

    /**
     * Sets limelight's LED state.
     *    0 = use the LED Mode set in the current pipeline.
     *    1 = force off.
     *    2 = force blink.
     *    3 = force on.
     * @param mode
     */
    public void setLedMode(int mode) {
        if (mode < 0 || mode > 3) {
            throw new IllegalArgumentException("LED mode must be between 0 and 3");
        }

        switch (mode) {
            case 0:
                LimelightHelpers.setLEDMode_PipelineControl(_limelightName);
                break;
            case 1:
                LimelightHelpers.setLEDMode_ForceOff(_limelightName);
                break;
            case 2:
                LimelightHelpers.setLEDMode_ForceBlink(_limelightName);
                break;
            case 3:
                LimelightHelpers.setLEDMode_ForceOn(_limelightName);
                break;
        }
    }

    /**
     * Forces the LED to blink a specified number of times, then returns to pipeline control.
     */
    public void blinkLed(int blinkCount) {
        m_executorService.submit(() -> {
            // Blink the LED X times with 100ms on, 200ms off for each blink
            for (int i = 0; i < blinkCount; i++) {
                LimelightHelpers.setLEDMode_ForceOn(_limelightName);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    Thread.currentThread().interrupt();
                }

                LimelightHelpers.setLEDMode_ForceOff(_limelightName);
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
     * Sets limelight's pipeline.
     * @param pipeline
     */
    public void setPipeline(int pipeline) {
        if (pipeline < 0 || pipeline > 9) {
            throw new IllegalArgumentException("Pipeline must be between 0 and 9");
        }

        LimelightHelpers.setPipelineIndex(_limelightName, pipeline);
    }

    /**
     * Side-by-side streams (Note: USB output stream is not affected by this mode)
     * @param mode
     */
    public void setPiPStreamingMode(int mode) {
        if (mode < 0 || mode > 2) {
            throw new IllegalArgumentException("Streaming mode must be between 0 and 2");
        }

        switch (mode) {
            case 0:
                LimelightHelpers.setStreamMode_Standard(_limelightName);
                break;
            case 1:
                LimelightHelpers.setStreamMode_PiPMain(_limelightName);
                break;
            case 2:
                LimelightHelpers.setStreamMode_PiPSecondary(_limelightName);
                break;
        }
    }

    /**
     * Set the camera's pose in the coordinate system of the robot.
     * @param pose
     */
    public void setCameraPose(Pose3d pose) {
        LimelightHelpers.setCameraPose_RobotSpace(_limelightName,
                pose.getTranslation().getX(),
                pose.getTranslation().getY(),
                pose.getTranslation().getZ(),
                Units.radiansToDegrees(pose.getRotation().getX()),
                Units.radiansToDegrees(pose.getRotation().getY()),
                Units.radiansToDegrees(pose.getRotation().getZ()));
    }

    //#endregion

    public void close() {
        m_executorService.shutdown();
    }
}
