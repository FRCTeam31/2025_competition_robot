package frc.robot.subsystems.vision.helpers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseEstimate implements LoggableInputs, Cloneable {
    public Pose2d pose;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;
    public RawFiducial[] rawFiducials;

    /**
     * Makes a PoseEstimate object with default values
     */
    public PoseEstimate() {
        this.pose = new Pose2d();
        this.timestampSeconds = 0;
        this.latency = 0;
        this.tagCount = 0;
        this.tagSpan = 0;
        this.avgTagDist = 0;
        this.avgTagArea = 0;
        this.rawFiducials = new RawFiducial[] {};
    }

    public PoseEstimate(Pose2d pose,
            double timestampSeconds,
            double latency,
            int tagCount,
            double tagSpan,
            double avgTagDist,
            double avgTagArea,
            RawFiducial[] rawFiducials) {

        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.latency = latency;
        this.tagCount = tagCount;
        this.tagSpan = tagSpan;
        this.avgTagDist = avgTagDist;
        this.avgTagArea = avgTagArea;
        this.rawFiducials = rawFiducials;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Pose", pose);
        table.put("TimestampSeconds", timestampSeconds);
        table.put("Latency", latency);
        table.put("TagCount", tagCount);
        table.put("TagSpan", tagSpan);
        table.put("AvgTagDist", avgTagDist);
        table.put("AvgTagArea", avgTagArea);
        // table.put("RawFiducials", rawFiducials);
    }

    @Override
    public void fromLog(LogTable table) {
        pose = table.get("Pose", pose);
        timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
        latency = table.get("Latency", latency);
        tagCount = table.get("TagCount", tagCount);
        tagSpan = table.get("TagSpan", tagSpan);
        avgTagDist = table.get("AvgTagDist", avgTagDist);
        avgTagArea = table.get("AvgTagArea", avgTagArea);
        // rawFiducials = table.get("RawFiducials", rawFiducials);
    }

    public PoseEstimate clone() {
        PoseEstimate copy = new PoseEstimate();
        copy.pose = this.pose;
        copy.timestampSeconds = this.timestampSeconds;
        copy.latency = this.latency;
        copy.tagCount = this.tagCount;
        copy.tagSpan = this.tagSpan;
        copy.avgTagDist = this.avgTagDist;
        copy.avgTagArea = this.avgTagArea;
        copy.rawFiducials = this.rawFiducials.clone();
        return copy;
    }
}
